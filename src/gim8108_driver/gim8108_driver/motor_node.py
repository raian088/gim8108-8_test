"""
GIM8108-8 Motor Driver Node (CAN / SLCAN)

CAN protocol: ODrive-compatible
  CAN ID = (node_id << 5) | cmd_id
  Standard 11-bit frame, 8 bytes, little-endian

Default interface: SLCAN (/dev/ttyACM0, 500 kbps) for USB-CAN adapters in WSL.
Switch to socketcan by setting can_bustype:='socketcan' and can_channel:='can0'.
"""

import math
import struct
import threading
import time

import can
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32
from std_srvs.srv import SetBool, Trigger

_QOS_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

# ── Axis states ──────────────────────────────────────────────────────────────
AXIS_STATE_IDLE = 1
AXIS_STATE_FULL_CALIBRATION = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# ── Control modes ─────────────────────────────────────────────────────────────
CTRL_TORQUE = 1
CTRL_VELOCITY = 2
CTRL_POSITION = 3

# ── Input modes ───────────────────────────────────────────────────────────────
INPUT_PASSTHROUGH = 1
INPUT_VEL_RAMP = 2
INPUT_POS_FILTER = 3
INPUT_TRAP_TRAJ = 5
INPUT_TORQUE_RAMP = 6

# ── CMD IDs ───────────────────────────────────────────────────────────────────
CMD_HEARTBEAT = 0x001
CMD_ESTOP = 0x002
CMD_SET_AXIS_STATE = 0x007
CMD_GET_ENCODER_ESTIMATES = 0x009
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_INPUT_POS = 0x00C
CMD_SET_INPUT_VEL = 0x00D
CMD_SET_INPUT_TORQUE = 0x00E
CMD_SET_LIMITS = 0x00F
CMD_SET_TRAJ_VEL_LIMIT = 0x011
CMD_SET_TRAJ_ACCEL_LIMITS = 0x012
CMD_GET_IQ = 0x014
CMD_CLEAR_ERRORS = 0x018
CMD_GET_TORQUES = 0x01C


def _make_id(node_id: int, cmd_id: int) -> int:
    return (node_id << 5) | cmd_id


def _parse_id(can_id: int):
    return (can_id >> 5) & 0x3F, can_id & 0x1F


class GIM8108MotorNode(Node):
    def __init__(self):
        super().__init__('gim8108_motor_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('can_channel', '/dev/ttyACM0')
        self.declare_parameter('can_bustype', 'slcan')
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('node_id', 0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('traj_accel', 5.0)   # turns/s²
        self.declare_parameter('traj_decel', 5.0)   # turns/s²
        self.declare_parameter('home_on_connect', True)

        channel = self.get_parameter('can_channel').value
        bustype = self.get_parameter('can_bustype').value
        bitrate = self.get_parameter('can_bitrate').value
        self.node_id = self.get_parameter('node_id').value

        # ── Internal state ────────────────────────────────────────────────────
        self.pos_est_turns = 0.0
        self.vel_est_turns = 0.0
        self.iq_measured = 0.0
        self.axis_state = 0
        self.axis_error = 0
        self._vel_limit = 10.0    # turns/s
        self._cur_limit = 7.0     # A
        self._traj_vel = 10.0     # turns/s (for TRAP_TRAJ)
        self._control_mode = CTRL_POSITION

        # ── CAN bus ───────────────────────────────────────────────────────────
        self.bus = None
        try:
            self.bus = can.interface.Bus(
                channel=channel,
                bustype=bustype,
                bitrate=bitrate,
            )
            self.get_logger().info(f'CAN bus connected: {bustype}:{channel}')
        except Exception as exc:
            self.get_logger().error(f'CAN bus failed: {exc}')
            self.get_logger().error('Set can_bustype and can_channel params correctly.')

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Float64, '~/cmd_pos_deg', self._on_cmd_pos, 10)
        self.create_subscription(Float64, '~/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Float64, '~/cmd_torque', self._on_cmd_torque, 10)
        self.create_subscription(Float64, '~/vel_limit', self._on_vel_limit, 10)
        self.create_subscription(Float64, '~/current_limit', self._on_cur_limit, 10)
        self.create_subscription(Float64, '~/traj_vel_limit', self._on_traj_vel, 10)
        self.create_subscription(Int32, '~/control_mode', self._on_mode, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_joint = self.create_publisher(JointState, '~/joint_state', 10)
        self.pub_connected = self.create_publisher(Bool, '~/is_connected', _QOS_LATCHED)

        # ── Services ──────────────────────────────────────────────────────────
        self.create_service(SetBool, '~/enable', self._srv_enable)
        self.create_service(Trigger, '~/clear_errors', self._srv_clear_errors)
        self.create_service(Trigger, '~/calibrate', self._srv_calibrate)
        self.create_service(Trigger, '~/estop', self._srv_estop)

        # ── CAN receive thread ────────────────────────────────────────────────
        t = threading.Thread(target=self._recv_loop, daemon=True)
        t.start()

        # ── Publish timer ─────────────────────────────────────────────────────
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._publish_state)

        # ── Home on connect ───────────────────────────────────────────────────
        if self.bus is not None and self.get_parameter('home_on_connect').value:
            threading.Thread(target=self._home_to_origin, daemon=True).start()

    # ── CAN helpers ───────────────────────────────────────────────────────────

    def _send(self, cmd_id: int, data: bytes):
        if self.bus is None:
            return
        payload = (data + b'\x00' * 8)[:8]
        msg = can.Message(
            arbitration_id=_make_id(self.node_id, cmd_id),
            data=payload,
            is_extended_id=False,
        )
        try:
            self.bus.send(msg)
        except Exception as exc:
            self.get_logger().warn(f'CAN send error: {exc}')

    def _recv_loop(self):
        while rclpy.ok():
            if self.bus is None:
                return
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg is None:
                    continue
                _, cmd_id = _parse_id(msg.arbitration_id)
                self._handle(cmd_id, bytes(msg.data))
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().warn(f'CAN recv error: {exc}', throttle_duration_sec=5.0)

    def _handle(self, cmd_id: int, data: bytes):
        if cmd_id == CMD_HEARTBEAT and len(data) >= 5:
            self.axis_error = struct.unpack_from('<I', data, 0)[0]
            self.axis_state = struct.unpack_from('<B', data, 4)[0]
        elif cmd_id == CMD_GET_ENCODER_ESTIMATES and len(data) >= 8:
            self.pos_est_turns, self.vel_est_turns = struct.unpack_from('<ff', data)
        elif cmd_id == CMD_GET_IQ and len(data) >= 8:
            _iq_set, self.iq_measured = struct.unpack_from('<ff', data)
        elif cmd_id == CMD_GET_TORQUES and len(data) >= 8:
            pass  # torque_setpoint, torque_measured

    # ── Command helpers ───────────────────────────────────────────────────────

    def _set_axis_state(self, state: int):
        self._send(CMD_SET_AXIS_STATE, struct.pack('<I', state))

    def _set_mode(self, ctrl: int, inp: int):
        self._send(CMD_SET_CONTROLLER_MODE, struct.pack('<II', ctrl, inp))

    def _update_limits(self):
        self._send(CMD_SET_LIMITS, struct.pack('<ff', self._vel_limit, self._cur_limit))

    def _update_traj_vel(self):
        self._send(CMD_SET_TRAJ_VEL_LIMIT, struct.pack('<f', self._traj_vel))

    def _home_to_origin(self):
        """CANバス接続後、自動的にクローズドループを有効化して原点(0°)へ移動する。"""
        self.get_logger().info('Homing: waiting for first heartbeat...')
        # ハートビートが届くまで最大5秒待つ
        for _ in range(50):
            time.sleep(0.1)
            if self.axis_state != 0:
                break
        else:
            self.get_logger().warn('Homing: no heartbeat received — motor may not be calibrated.')

        self.get_logger().info('Homing: enabling closed-loop and moving to 0°...')
        self._set_mode(CTRL_POSITION, INPUT_TRAP_TRAJ)
        self._update_limits()
        self._update_traj_vel()
        accel = self.get_parameter('traj_accel').value
        decel = self.get_parameter('traj_decel').value
        self._send(CMD_SET_TRAJ_ACCEL_LIMITS, struct.pack('<ff', accel, decel))
        self._set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(0.1)
        self._send(CMD_SET_INPUT_POS, struct.pack('<fhh', 0.0, 0, 0))
        self.get_logger().info('Homing: command sent → 0°')

    # ── Subscription callbacks ─────────────────────────────────────────────────

    def _on_cmd_pos(self, msg: Float64):
        pos_turns = msg.data / 360.0
        self._send(CMD_SET_INPUT_POS, struct.pack('<fhh', pos_turns, 0, 0))

    def _on_cmd_vel(self, msg: Float64):
        self._send(CMD_SET_INPUT_VEL, struct.pack('<ff', msg.data, 0.0))

    def _on_cmd_torque(self, msg: Float64):
        self._send(CMD_SET_INPUT_TORQUE, struct.pack('<f', msg.data))

    def _on_vel_limit(self, msg: Float64):
        self._vel_limit = float(msg.data)
        self._update_limits()

    def _on_cur_limit(self, msg: Float64):
        self._cur_limit = float(msg.data)
        self._update_limits()

    def _on_traj_vel(self, msg: Float64):
        self._traj_vel = float(msg.data)
        self._update_traj_vel()
        accel = self.get_parameter('traj_accel').value
        decel = self.get_parameter('traj_decel').value
        self._send(CMD_SET_TRAJ_ACCEL_LIMITS, struct.pack('<ff', accel, decel))

    def _on_mode(self, msg: Int32):
        mode = msg.data
        self._control_mode = mode
        if mode == CTRL_POSITION:
            self._set_mode(CTRL_POSITION, INPUT_TRAP_TRAJ)
        elif mode == CTRL_VELOCITY:
            self._set_mode(CTRL_VELOCITY, INPUT_VEL_RAMP)
        elif mode == CTRL_TORQUE:
            self._set_mode(CTRL_TORQUE, INPUT_TORQUE_RAMP)

    # ── Services ──────────────────────────────────────────────────────────────

    def _srv_enable(self, request, response):
        if request.data:
            self._set_mode(CTRL_POSITION, INPUT_TRAP_TRAJ)
            self._update_limits()
            self._update_traj_vel()
            self._set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
            response.message = 'Motor enabled (closed-loop position control)'
        else:
            self._set_axis_state(AXIS_STATE_IDLE)
            response.message = 'Motor disabled (IDLE)'
        response.success = True
        return response

    def _srv_clear_errors(self, request, response):
        self._send(CMD_CLEAR_ERRORS, b'\x00' * 8)
        response.success = True
        response.message = 'Errors cleared'
        return response

    def _srv_calibrate(self, request, response):
        self.get_logger().info('Starting full calibration (motor + encoder)…')
        self._set_axis_state(AXIS_STATE_FULL_CALIBRATION)
        response.success = True
        response.message = 'Calibration sequence started (takes ~10 s)'
        return response

    def _srv_estop(self, request, response):
        self._send(CMD_ESTOP, b'\x00' * 8)
        response.success = True
        response.message = 'E-stop sent'
        return response

    # ── State publisher ────────────────────────────────────────────────────────

    def _publish_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['gim8108_joint']
        js.position = [self.pos_est_turns * 2.0 * math.pi]   # rad
        js.velocity = [self.vel_est_turns * 2.0 * math.pi]   # rad/s
        js.effort = [self.iq_measured]                         # A
        self.pub_joint.publish(js)

        connected = Bool()
        connected.data = self.bus is not None
        self.pub_connected.publish(connected)


def main(args=None):
    rclpy.init(args=args)
    node = GIM8108MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
