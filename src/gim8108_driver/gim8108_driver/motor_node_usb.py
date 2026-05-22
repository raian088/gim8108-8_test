"""
GIM8108-8 Motor Driver Node — USB (ODrive SDK)
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32, String
from std_srvs.srv import SetBool, Trigger

# Latched QoS: late-joining subscribers receive the last message immediately
_QOS_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

try:
    import odrive
    from odrive.enums import ControlMode, InputMode
    ODRIVE_AVAILABLE = True
except ImportError:
    ODRIVE_AVAILABLE = False

# Axis states (integer values — compatible with any firmware version)
IDLE = 1
MOTOR_CALIB = 4
ENC_CALIB = 7
CLOSED_LOOP = 8

CTRL_TORQUE = 1
CTRL_VELOCITY = 2
CTRL_POSITION = 3


class GIM8108UsbNode(Node):
    def __init__(self):
        super().__init__('gim8108_motor_node')

        self.declare_parameter('axis',            0)
        self.declare_parameter('publish_rate',    50.0)
        self.declare_parameter('connect_timeout', 60.0)
        self.declare_parameter('traj_vel',        10.0)
        self.declare_parameter('traj_accel',      5.0)
        self.declare_parameter('traj_decel',      5.0)
        self.declare_parameter('gear_ratio',      8.0)
        self.declare_parameter('home_on_connect', True)
        self.declare_parameter('serial_number',   '')   # hex string, e.g. '3060649C3539' — blank = auto
        self.declare_parameter('usb_path',        '')   # 'bus:address' e.g. '001:017' from lsusb — overrides serial_number

        self.odrv = None
        self.axis = None
        self._usb_find_path = None   # set in _connect_loop if usb_path param given
        self._lock = threading.Lock()
        self._traj_vel   = self.get_parameter('traj_vel').value
        self._traj_accel = self.get_parameter('traj_accel').value
        self._traj_decel = self.get_parameter('traj_decel').value
        self._cur_limit  = 7.0
        self._vel_limit  = 30.0
        self._gear_ratio = self.get_parameter('gear_ratio').value
        self._fail_count = 0          # consecutive read failures
        self._reconnecting = False    # reconnect thread guard
        self._save_in_progress = False  # suppress disconnect during save/reboot
        self._FAIL_THRESHOLD = 5      # failures before declaring disconnected
        self._last_conn_state = None  # last published is_connected value

        # Subscriptions
        self.create_subscription(Float64, '~/cmd_pos_deg',        self._on_pos,           10)
        self.create_subscription(Float64, '~/cmd_vel',             self._on_vel,           10)
        self.create_subscription(Float64, '~/cmd_torque',          self._on_torque,        10)
        self.create_subscription(Float64, '~/vel_limit',           self._on_vel_limit,     10)
        self.create_subscription(Float64, '~/current_limit',       self._on_cur_limit,     10)
        self.create_subscription(Float64, '~/traj_vel_limit',      self._on_traj_vel,      10)
        self.create_subscription(Float64, '~/traj_accel_limit',    self._on_traj_accel,    10)
        self.create_subscription(Float64, '~/traj_decel_limit',    self._on_traj_decel,    10)
        self.create_subscription(Int32,   '~/control_mode',        self._on_mode,          10)
        self.create_subscription(Float64, '~/pos_gain',            self._on_pos_gain,      10)
        self.create_subscription(Float64, '~/vel_gain',            self._on_vel_gain,      10)
        self.create_subscription(Float64, '~/vel_integrator_gain', self._on_vel_int_gain,  10)

        # Publishers
        self.pub_joint   = self.create_publisher(JointState, '~/joint_state',  10)
        self.pub_conn    = self.create_publisher(Bool,        '~/is_connected', _QOS_LATCHED)
        self.pub_calib   = self.create_publisher(String,      '~/calib_status', 10)
        self.pub_voltage = self.create_publisher(Float64,     '~/bus_voltage',  10)
        # Config feedback — latched so GUI gets current values even if started later
        self.pub_cfg_pos_gain   = self.create_publisher(Float64, '~/config/pos_gain',            _QOS_LATCHED)
        self.pub_cfg_vel_gain   = self.create_publisher(Float64, '~/config/vel_gain',            _QOS_LATCHED)
        self.pub_cfg_vel_int    = self.create_publisher(Float64, '~/config/vel_integrator_gain', _QOS_LATCHED)
        self.pub_cfg_traj_vel   = self.create_publisher(Float64, '~/config/traj_vel_limit',      _QOS_LATCHED)
        self.pub_cfg_traj_accel = self.create_publisher(Float64, '~/config/traj_accel_limit',    _QOS_LATCHED)
        self.pub_cfg_traj_decel = self.create_publisher(Float64, '~/config/traj_decel_limit',    _QOS_LATCHED)
        self.pub_cfg_vel_limit  = self.create_publisher(Float64, '~/config/vel_limit',           _QOS_LATCHED)
        self.pub_cfg_cur_limit  = self.create_publisher(Float64, '~/config/current_lim',         _QOS_LATCHED)
        self.pub_serial         = self.create_publisher(String,  '~/serial_number',              _QOS_LATCHED)

        # Services
        self.create_service(SetBool, '~/enable',       self._srv_enable)
        self.create_service(Trigger, '~/clear_errors', self._srv_clear_errors)
        self.create_service(Trigger, '~/calibrate',    self._srv_calibrate)
        self.create_service(Trigger, '~/save_config',  self._srv_save_config)
        self.create_service(Trigger, '~/estop',        self._srv_estop)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._publish_state)

        threading.Thread(target=self._connect_loop, daemon=True).start()

    # ── Connection ────────────────────────────────────────────────────────────

    def _connect_loop(self):
        """Initial connection — called once at startup."""
        if not ODRIVE_AVAILABLE:
            self.get_logger().error('odrive package not found: pip install odrive')
            return
        timeout = self.get_parameter('connect_timeout').value
        axis_num = self.get_parameter('axis').value
        usb_path = self.get_parameter('usb_path').value      # e.g. '001:017'
        sn_str   = self.get_parameter('serial_number').value  # e.g. '3060649C3539'
        sn = None

        if usb_path:
            # Pass USB bus:address directly to odrive as a path filter.
            # Format: 'usb:bus:address' e.g. 'usb:1:17' (decimal)
            try:
                bus, addr = (int(x) for x in usb_path.split(':'))
                self._usb_find_path = f'usb:{bus}:{addr}'
                self.get_logger().info(f'USB path filter: {self._usb_find_path}')
            except Exception as exc:
                self.get_logger().warn(f'USB path parse failed ({exc}) — auto-detecting')
                self._usb_find_path = None
        elif sn_str:
            try:
                sn = int(sn_str, 16)
            except ValueError:
                self.get_logger().warn(
                    f'serial_number "{sn_str}" is not valid hex — falling back to auto-detect')

        ident = usb_path or sn_str or 'auto'
        self.get_logger().info(
            f'Waiting for ODrive (timeout={timeout}s, id={ident}) — '
            'make sure: usbipd attach --wsl --busid <BUSID>'
        )
        try:
            if self._usb_find_path:
                odrv = odrive.find_any(timeout=timeout, path=self._usb_find_path)
            else:
                odrv = odrive.find_any(timeout=timeout, serial_number=sn)
        except Exception as exc:
            self.get_logger().error(f'ODrive connection failed: {exc}')
            return
        with self._lock:
            self.odrv = odrv
            self.axis = odrv.axis0 if axis_num == 0 else odrv.axis1
            self._fail_count = 0
        sn_hex = hex(odrv.serial_number)[2:].upper()
        self.get_logger().info(f'ODrive connected!  Serial: {sn_hex}')
        self.pub_serial.publish(String(data=sn_hex))
        self._publish_config()
        if self.get_parameter('home_on_connect').value:
            threading.Thread(target=self._home_to_origin, daemon=True).start()

    def _reconnect_loop(self):
        """Called automatically when reads fail; keeps retrying until success."""
        axis_num = self.get_parameter('axis').value
        self.get_logger().info('ODrive disconnected — waiting to reconnect...')
        while rclpy.ok():
            try:
                odrv = odrive.find_any(timeout=10.0)
            except Exception:
                time.sleep(2.0)
                continue
            with self._lock:
                self.odrv = odrv
                self.axis = odrv.axis0 if axis_num == 0 else odrv.axis1
                self._fail_count = 0
                self._reconnecting = False
                self._save_in_progress = False
            sn_hex = hex(odrv.serial_number)[2:].upper()
            self.get_logger().info(f'ODrive reconnected!  Serial: {sn_hex}')
            self.pub_serial.publish(String(data=sn_hex))
            self._publish_config()
            return

    def _home_to_origin(self):
        """接続後、自動的に原点(0°)へ移動する。"""
        self.get_logger().info('Homing: enabling closed-loop and moving to 0°...')
        with self._lock:
            if not self._connected:
                return
            try:
                self.axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
                self.axis.controller.config.input_mode   = InputMode.TRAP_TRAJ
                self.axis.trap_traj.config.vel_limit    = self._traj_vel
                self.axis.trap_traj.config.accel_limit  = self._traj_accel
                self.axis.trap_traj.config.decel_limit  = self._traj_decel
                self.axis.controller.config.vel_limit   = self._vel_limit
                self.axis.motor.config.current_lim      = self._cur_limit
                self.axis.requested_state = CLOSED_LOOP
                time.sleep(0.1)
                self.axis.controller.input_pos = 0.0
                self.get_logger().info('Homing: command sent → 0°')
            except Exception as exc:
                self.get_logger().error(
                    f'Homing failed (motor may not be calibrated yet): {exc}'
                )

    @property
    def _connected(self) -> bool:
        return self.axis is not None

    # ── Subscription callbacks ─────────────────────────────────────────────────

    def _on_pos(self, msg: Float64):
        with self._lock:
            if not self._connected: return
            try:
                # Convert output-shaft degrees → motor-shaft turns (accounts for gear ratio)
                self.axis.controller.input_pos = (msg.data / 360.0) * self._gear_ratio
            except Exception as exc:
                self.get_logger().warn(f'cmd_pos error: {exc}', throttle_duration_sec=2.0)

    def _on_vel(self, msg: Float64):
        with self._lock:
            if not self._connected: return
            try:
                self.axis.controller.input_vel = msg.data
            except Exception as exc:
                self.get_logger().warn(f'cmd_vel error: {exc}', throttle_duration_sec=2.0)

    def _on_torque(self, msg: Float64):
        with self._lock:
            if not self._connected: return
            try:
                self.axis.controller.input_torque = msg.data
            except Exception as exc:
                self.get_logger().warn(f'cmd_torque error: {exc}', throttle_duration_sec=2.0)

    def _on_vel_limit(self, msg: Float64):
        self._vel_limit = float(msg.data)
        with self._lock:
            if not self._connected: return
            try:
                self.axis.controller.config.vel_limit = self._vel_limit
            except Exception as exc:
                self.get_logger().warn(f'vel_limit error: {exc}', throttle_duration_sec=2.0)

    def _on_cur_limit(self, msg: Float64):
        self._cur_limit = float(msg.data)
        with self._lock:
            if not self._connected: return
            try:
                self.axis.motor.config.current_lim = self._cur_limit
            except Exception as exc:
                self.get_logger().warn(f'current_lim error: {exc}', throttle_duration_sec=2.0)

    def _on_traj_vel(self, msg: Float64):
        self._traj_vel = float(msg.data)
        with self._lock:
            if not self._connected: return
            try:
                self.axis.trap_traj.config.vel_limit = self._traj_vel
            except Exception as exc:
                self.get_logger().warn(f'traj_vel error: {exc}', throttle_duration_sec=2.0)

    def _on_traj_accel(self, msg: Float64):
        self._traj_accel = float(msg.data)
        with self._lock:
            if not self._connected: return
            try:
                self.axis.trap_traj.config.accel_limit = self._traj_accel
            except Exception as exc:
                self.get_logger().warn(f'traj_accel error: {exc}', throttle_duration_sec=2.0)

    def _on_traj_decel(self, msg: Float64):
        self._traj_decel = float(msg.data)
        with self._lock:
            if not self._connected: return
            try:
                self.axis.trap_traj.config.decel_limit = self._traj_decel
            except Exception as exc:
                self.get_logger().warn(f'traj_decel error: {exc}', throttle_duration_sec=2.0)

    def _on_mode(self, msg: Int32):
        with self._lock:
            if not self._connected: return
            try:
                mode = msg.data
                if mode == CTRL_POSITION:
                    self.axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
                    self.axis.controller.config.input_mode   = InputMode.TRAP_TRAJ
                elif mode == CTRL_VELOCITY:
                    self.axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
                    self.axis.controller.config.input_mode   = InputMode.VEL_RAMP
                elif mode == CTRL_TORQUE:
                    self.axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
                    self.axis.controller.config.input_mode   = InputMode.TORQUE_RAMP
            except Exception as exc:
                self.get_logger().warn(f'mode error: {exc}', throttle_duration_sec=2.0)

    def _on_pos_gain(self, msg: Float64):
        with self._lock:
            if not self._connected: return
            try:
                self.axis.controller.config.pos_gain = msg.data
            except Exception as exc:
                self.get_logger().warn(f'pos_gain error: {exc}', throttle_duration_sec=2.0)

    def _on_vel_gain(self, msg: Float64):
        with self._lock:
            if not self._connected: return
            try:
                self.axis.controller.config.vel_gain = msg.data
            except Exception as exc:
                self.get_logger().warn(f'vel_gain error: {exc}', throttle_duration_sec=2.0)

    def _on_vel_int_gain(self, msg: Float64):
        with self._lock:
            if not self._connected: return
            try:
                self.axis.controller.config.vel_integrator_gain = msg.data
            except Exception as exc:
                self.get_logger().warn(f'vel_integrator_gain error: {exc}', throttle_duration_sec=2.0)

    # ── Services ──────────────────────────────────────────────────────────────

    def _srv_enable(self, request, response):
        with self._lock:
            if not self._connected:
                response.success = False
                response.message = 'ODrive not connected'
                return response
            try:
                if request.data:
                    self.axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
                    self.axis.controller.config.input_mode   = InputMode.TRAP_TRAJ
                    self.axis.trap_traj.config.vel_limit    = self._traj_vel
                    self.axis.trap_traj.config.accel_limit  = self._traj_accel
                    self.axis.trap_traj.config.decel_limit  = self._traj_decel
                    self.axis.controller.config.vel_limit   = self._vel_limit
                    self.axis.motor.config.current_lim      = self._cur_limit
                    self.axis.requested_state = CLOSED_LOOP
                    response.message = 'Motor enabled (position / TRAP_TRAJ)'
                else:
                    self.axis.requested_state = IDLE
                    response.message = 'Motor disabled (IDLE)'
                response.success = True
            except Exception as exc:
                response.success = False
                response.message = str(exc)
        return response

    def _srv_clear_errors(self, request, response):
        with self._lock:
            if not self._connected:
                response.success = False
                response.message = 'ODrive not connected'
                return response
            try:
                self.odrv.clear_errors()
                response.success = True
                response.message = 'Errors cleared'
            except Exception as exc:
                response.success = False
                response.message = str(exc)
        return response

    def _srv_calibrate(self, request, response):
        with self._lock:
            if not self._connected:
                response.success = False
                response.message = 'ODrive not connected'
                return response
        threading.Thread(target=self._run_calibration, daemon=True).start()
        response.success = True
        response.message = 'Calibration started (runs in background ~25s)'
        return response

    def _run_calibration(self):
        def publish_status(msg: str):
            self.get_logger().info(f'[Calibration] {msg}')
            s = String(); s.data = msg
            self.pub_calib.publish(s)

        def explain_error(code: int) -> str:
            known = {
                0x00000001: 'PHASE_RESISTANCE_OUT_OF_RANGE — increase calibration_current or check wiring',
                0x00000002: 'PHASE_INDUCTANCE_OUT_OF_RANGE — check motor wiring',
                0x00000004: 'ADC_FAILED — hardware issue on driver board',
                0x00000008: 'DRV_FAULT — gate driver fault, check motor wiring and power connections',
                0x00000010: 'CONTROL_DEADLINE_MISSED',
                0x00000020: 'NOT_IMPLEMENTED_MOTOR_TYPE — motor_type config mismatch',
                0x00000080: 'MODULATION_MAGNITUDE — bus voltage too low for this current target',
                0x00000400: 'CURRENT_MEASUREMENT_TIMEOUT',
                0x00001000: 'CURRENT_SENSE_SATURATION',
                0x00004000: 'CURRENT_LIMIT_VIOLATION',
                0x01000000: 'SYSTEM_LEVEL / ESTOP_REQUESTED — hardware safety triggered; '
                            'check motor phase wiring, try erase_configuration if persistent',
            }
            parts = [name for mask, name in known.items() if code & mask]
            return '; '.join(parts) if parts else f'unknown (0x{code:08X})'

        with self._lock:
            axis = self.axis
            odrv = self.odrv

        try:
            # ── Step 0: Check bus voltage ─────────────────────────────────────
            publish_status('Checking power supply...')
            try:
                vbus = odrv.vbus_voltage
            except Exception as exc:
                publish_status(f'WARNING: Could not read vbus_voltage: {exc}')
                vbus = None

            if vbus is not None:
                if vbus < 10.0:
                    publish_status(
                        f'ABORTED: Bus voltage {vbus:.1f}V — '
                        'connect XT30 main power (15-60V DC) first.'
                    )
                    return
                elif vbus < 15.0:
                    publish_status(
                        f'WARNING: Bus voltage {vbus:.1f}V is below rated minimum (15V). '
                        'Calibration may fail — use a ≥15V supply if it does.'
                    )
                else:
                    publish_status(f'Bus voltage: {vbus:.1f} V — OK')

            # ── Step 1: Force IDLE, then clear errors ─────────────────────────
            publish_status('Setting IDLE state and clearing errors...')
            try:
                axis.requested_state = IDLE
            except Exception:
                pass
            time.sleep(0.3)
            try:
                odrv.clear_errors()
            except Exception:
                pass
            time.sleep(0.5)

            # ── Step 2: Log current motor_type for diagnostics ────────────────
            try:
                cur_type = axis.motor.config.motor_type
                type_name = {0: 'HIGH_CURRENT', 2: 'GIMBAL'}.get(cur_type, 'UNKNOWN')
                publish_status(f'Motor type: {cur_type} ({type_name})')
            except Exception:
                pass

            # ── Step 3: Apply GIM8108-8 motor parameters ──────────────────────
            publish_status('Setting GIM8108-8 motor parameters (pole_pairs=21, cal_current=10A)...')
            try:
                axis.motor.config.pole_pairs          = 21
                axis.motor.config.torque_constant      = 1.0
                axis.motor.config.current_lim          = 30.0
                axis.motor.config.calibration_current  = 10.0
                # Do NOT set motor_type — preserve factory default
            except Exception as exc:
                publish_status(f'WARNING: Could not set some motor parameters: {exc}')

            # ── Step 4: Motor calibration ─────────────────────────────────────
            publish_status('Step 1/2: Motor calibration (listen for beep)...')
            axis.requested_state = MOTOR_CALIB

            for _ in range(300):   # wait up to 30 s
                time.sleep(0.1)
                try:
                    if axis.current_state == IDLE:
                        break
                except Exception:
                    break

            try:
                motor_err = axis.motor.error
            except Exception:
                motor_err = 0
            try:
                axis_err = axis.error
            except Exception:
                axis_err = 0

            if motor_err != 0 or axis_err != 0:
                parts = []
                if motor_err:
                    parts.append(f'motor=0x{motor_err:08X} ({explain_error(motor_err)})')
                if axis_err:
                    parts.append(f'axis=0x{axis_err:08X}')
                publish_status(f'FAILED: {", ".join(parts)}')
                return

            # ── Step 5: Encoder offset calibration ───────────────────────────
            publish_status('Step 2/2: Encoder offset calibration (motor will rotate slightly)...')
            axis.requested_state = ENC_CALIB

            for _ in range(200):   # wait up to 20 s
                time.sleep(0.1)
                try:
                    if axis.current_state == IDLE:
                        break
                except Exception:
                    break

            try:
                enc_err = axis.encoder.error
            except Exception:
                enc_err = 0

            if enc_err != 0:
                publish_status(f'FAILED: encoder error=0x{enc_err:08X} ({explain_error(enc_err)})')
                return

            # ── Step 6: Mark as pre-calibrated (do NOT save yet) ─────────────
            try:
                axis.motor.config.pre_calibrated   = True
                axis.encoder.config.pre_calibrated = True
            except Exception as exc:
                publish_status(f'Note: pre_calibrated flag error (may be normal): {exc}')

            publish_status(
                'Calibration COMPLETE! '
                'Click "Save Config" to persist (ODrive will reboot).'
            )

        except Exception as exc:
            publish_status(f'Calibration ERROR: {exc}')

    def _srv_save_config(self, request, response):
        """Save configuration to ODrive flash (triggers USB disconnect + reboot)."""
        with self._lock:
            if not self._connected:
                response.success = False
                response.message = 'ODrive not connected'
                return response
            try:
                self._save_in_progress = True  # suppress is_connected=False during reboot
                self.odrv.save_configuration()
                self.axis = None
                self.odrv = None
            except Exception:
                pass  # disconnect exception is expected after save

        self.get_logger().info('Config saved. ODrive rebooting — reconnecting...')
        self._reconnecting = True
        threading.Thread(target=self._reconnect_loop, daemon=True).start()
        response.success = True
        response.message = 'Config saved. Reconnecting...'
        return response

    def _srv_estop(self, request, response):
        with self._lock:
            if not self._connected:
                response.success = False
                response.message = 'ODrive not connected'
                return response
            try:
                self.axis.requested_state = IDLE
                response.success = True
                response.message = 'E-Stop: motor set to IDLE'
            except Exception as exc:
                response.success = False
                response.message = str(exc)
        return response

    # ── State publisher ────────────────────────────────────────────────────────

    def _publish_state(self):
        with self._lock:
            if not self._connected:
                self._pub_conn_if_changed(False)
                return

            # ── Critical reads: pos / vel only ───────────────────────────────
            try:
                pos_turns = self.axis.encoder.pos_estimate
                vel_turns = self.axis.encoder.vel_estimate
                self._fail_count = 0
            except Exception as exc:
                self._fail_count += 1
                if self._fail_count >= self._FAIL_THRESHOLD:
                    self.get_logger().warn(
                        f'ODrive disconnected (encoder read failed {self._fail_count}x): {exc}'
                    )
                    self.axis = None
                    self.odrv = None
                    self._fail_count = 0
                    if not self._reconnecting:
                        self._reconnecting = True
                        threading.Thread(
                            target=self._reconnect_loop, daemon=True
                        ).start()
                    self._pub_conn_if_changed(False)
                # transient error → keep Connected, don't publish state
                return

            # ── Non-critical reads: current + voltage ─────────────────────────
            try:
                iq = self.axis.motor.current_control.Iq_measured
            except Exception:
                iq = 0.0
            try:
                vbus = self.odrv.vbus_voltage
            except Exception:
                vbus = 0.0

        # Successful read → publish
        self._pub_conn_if_changed(True)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ['gim8108_joint']
        # Divide by gear_ratio to report output-shaft position/velocity
        js.position = [(pos_turns / self._gear_ratio) * 2.0 * math.pi]
        js.velocity = [(vel_turns / self._gear_ratio) * 2.0 * math.pi]
        js.effort   = [iq]
        self.pub_joint.publish(js)

        v_msg = Float64()
        v_msg.data = vbus
        self.pub_voltage.publish(v_msg)

    def _publish_config(self):
        """ODrive のフラッシュ値を読み戻してローカル変数と GUI を同期する。"""
        with self._lock:
            if not self._connected:
                return
            try:
                pg  = self.axis.controller.config.pos_gain
                vg  = self.axis.controller.config.vel_gain
                vig = self.axis.controller.config.vel_integrator_gain
                tv  = self.axis.trap_traj.config.vel_limit
                ta  = self.axis.trap_traj.config.accel_limit
                td  = self.axis.trap_traj.config.decel_limit
                vl  = self.axis.controller.config.vel_limit
                cl  = self.axis.motor.config.current_lim
            except Exception as exc:
                self.get_logger().warn(f'Could not read config from ODrive: {exc}')
                return

        # ローカル変数を ODrive の保存値で上書き（enable/home 時に正しい値を使うため）
        self._traj_vel   = tv
        self._traj_accel = ta
        self._traj_decel = td
        self._vel_limit  = vl
        self._cur_limit  = cl

        def pub(publisher, value):
            m = Float64(); m.data = float(value); publisher.publish(m)

        pub(self.pub_cfg_pos_gain,   pg)
        pub(self.pub_cfg_vel_gain,   vg)
        pub(self.pub_cfg_vel_int,    vig)
        pub(self.pub_cfg_traj_vel,   tv)
        pub(self.pub_cfg_traj_accel, ta)
        pub(self.pub_cfg_traj_decel, td)
        pub(self.pub_cfg_vel_limit,  vl)
        pub(self.pub_cfg_cur_limit,  cl)
        self.get_logger().info(
            f'Config synced from ODrive: pos_gain={pg:.1f} vel_gain={vg:.3f} '
            f'vel_int={vig:.3f} traj_vel={tv:.1f} accel={ta:.1f} decel={td:.1f} '
            f'vel_limit={vl:.1f} cur_limit={cl:.1f}'
        )

    def _pub_conn_if_changed(self, state: bool):
        """Publish is_connected only when the value actually changes.
        Suppresses False during save_configuration reboot so GUI stays green."""
        if not state and self._save_in_progress:
            return
        if state != self._last_conn_state:
            self._last_conn_state = state
            msg = Bool()
            msg.data = state
            self.pub_conn.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GIM8108UsbNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
