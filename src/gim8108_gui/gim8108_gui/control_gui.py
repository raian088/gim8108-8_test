"""
GIM8108-8 Motor Control GUI
- All text in English (avoids font issues in WSL)
- Calibration status display + Save Config button
"""

import math
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32, String
from std_srvs.srv import SetBool, Trigger

_QOS_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QSlider,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

CTRL_TORQUE   = 1
CTRL_VELOCITY = 2
CTRL_POSITION = 3

DRIVER_NS = '/gim8108_motor_node'


class GuiNode(Node):
    def __init__(self):
        super().__init__('gim8108_gui')

        self.pub_pos         = self.create_publisher(Float64, f'{DRIVER_NS}/cmd_pos_deg',        10)
        self.pub_vel         = self.create_publisher(Float64, f'{DRIVER_NS}/cmd_vel',             10)
        self.pub_torque      = self.create_publisher(Float64, f'{DRIVER_NS}/cmd_torque',          10)
        self.pub_vel_lim     = self.create_publisher(Float64, f'{DRIVER_NS}/vel_limit',           10)
        self.pub_cur_lim     = self.create_publisher(Float64, f'{DRIVER_NS}/current_limit',       10)
        self.pub_traj_vel    = self.create_publisher(Float64, f'{DRIVER_NS}/traj_vel_limit',      10)
        self.pub_mode        = self.create_publisher(Int32,   f'{DRIVER_NS}/control_mode',        10)
        self.pub_pos_gain     = self.create_publisher(Float64, f'{DRIVER_NS}/pos_gain',            10)
        self.pub_vel_gain     = self.create_publisher(Float64, f'{DRIVER_NS}/vel_gain',            10)
        self.pub_vel_int_gain = self.create_publisher(Float64, f'{DRIVER_NS}/vel_integrator_gain', 10)
        self.pub_traj_accel   = self.create_publisher(Float64, f'{DRIVER_NS}/traj_accel_limit',    10)
        self.pub_traj_decel   = self.create_publisher(Float64, f'{DRIVER_NS}/traj_decel_limit',    10)

        self.create_subscription(JointState, f'{DRIVER_NS}/joint_state',  self._on_state,        10)
        self.create_subscription(Bool,       f'{DRIVER_NS}/is_connected', self._on_connected,    _QOS_LATCHED)
        self.create_subscription(String,     f'{DRIVER_NS}/calib_status', self._on_calib_status, 10)
        self.create_subscription(Float64,    f'{DRIVER_NS}/bus_voltage',  self._on_voltage,      10)
        # Config feedback — receive actual ODrive values after connect/reconnect
        ns = DRIVER_NS
        self.create_subscription(Float64, f'{ns}/config/pos_gain',            self._on_cfg('pos_gain'),   _QOS_LATCHED)
        self.create_subscription(Float64, f'{ns}/config/vel_gain',            self._on_cfg('vel_gain'),   _QOS_LATCHED)
        self.create_subscription(Float64, f'{ns}/config/vel_integrator_gain', self._on_cfg('vel_int'),    _QOS_LATCHED)
        self.create_subscription(Float64, f'{ns}/config/traj_vel_limit',      self._on_cfg('traj_vel'),   _QOS_LATCHED)
        self.create_subscription(Float64, f'{ns}/config/traj_accel_limit',    self._on_cfg('traj_accel'), _QOS_LATCHED)
        self.create_subscription(Float64, f'{ns}/config/traj_decel_limit',    self._on_cfg('traj_decel'), _QOS_LATCHED)

        self.enable_cli = self.create_client(SetBool, f'{DRIVER_NS}/enable')
        self.clear_cli  = self.create_client(Trigger, f'{DRIVER_NS}/clear_errors')
        self.calib_cli  = self.create_client(Trigger, f'{DRIVER_NS}/calibrate')
        self.save_cli   = self.create_client(Trigger, f'{DRIVER_NS}/save_config')
        self.estop_cli  = self.create_client(Trigger, f'{DRIVER_NS}/estop')

        self.pos_deg      = 0.0
        self.vel_out_ts   = 0.0   # output-shaft turns/s
        self.current_a    = 0.0
        self.voltage_v    = None  # None until first reading
        self.is_connected = False
        self.calib_msg    = ''
        self._cfg         = {}   # latest config values from driver

        self.on_state      = None
        self.on_connected  = None
        self.on_calib      = None
        self.on_config     = None   # called with (key, value)

    def _on_state(self, msg: JointState):
        if msg.position:
            # Output-shaft degrees, wrapped 0-360
            raw = math.degrees(msg.position[0]) % 360.0
            self.pos_deg = raw
        if msg.velocity:
            # Output-shaft turns/s  (rad/s ÷ 2π)
            self.vel_out_ts = msg.velocity[0] / (2.0 * math.pi)
        if msg.effort:
            self.current_a = msg.effort[0]
        if self.on_state: self.on_state()

    def _on_voltage(self, msg: Float64):
        self.voltage_v = msg.data
        if self.on_state: self.on_state()

    def _on_cfg(self, key: str):
        def _cb(msg: Float64):
            self._cfg[key] = msg.data
            if self.on_config:
                self.on_config(key, msg.data)
        return _cb

    def _on_connected(self, msg: Bool):
        if self.is_connected != msg.data:
            self.is_connected = msg.data
            if self.on_connected:
                self.on_connected()

    def _on_calib_status(self, msg: String):
        self.calib_msg = msg.data
        if self.on_calib: self.on_calib(msg.data)

    # ── Publish helpers ───────────────────────────────────────────────────────

    def send_pos_deg(self, deg: float):
        m = Float64(); m.data = deg; self.pub_pos.publish(m)

    def send_vel(self, v: float):
        m = Float64(); m.data = v; self.pub_vel.publish(m)

    def send_torque(self, t: float):
        m = Float64(); m.data = t; self.pub_torque.publish(m)

    def send_vel_limit(self, v: float):
        m = Float64(); m.data = v; self.pub_vel_lim.publish(m)

    def send_cur_limit(self, a: float):
        m = Float64(); m.data = a; self.pub_cur_lim.publish(m)

    def send_traj_vel(self, v: float):
        m = Float64(); m.data = v; self.pub_traj_vel.publish(m)

    def send_mode(self, mode: int):
        m = Int32(); m.data = mode; self.pub_mode.publish(m)

    def send_pos_gain(self, v: float):
        m = Float64(); m.data = v; self.pub_pos_gain.publish(m)

    def send_vel_gain(self, v: float):
        m = Float64(); m.data = v; self.pub_vel_gain.publish(m)

    def send_vel_int_gain(self, v: float):
        m = Float64(); m.data = v; self.pub_vel_int_gain.publish(m)

    def send_traj_accel(self, v: float):
        m = Float64(); m.data = v; self.pub_traj_accel.publish(m)

    def send_traj_decel(self, v: float):
        m = Float64(); m.data = v; self.pub_traj_decel.publish(m)

    def _call(self, client, request, label: str):
        if not client.service_is_ready():
            return f'{label}: driver not ready'
        client.call_async(request)
        return None

    def call_enable(self, enable: bool):
        req = SetBool.Request(); req.data = enable
        return self._call(self.enable_cli, req, 'enable')

    def call_clear(self):
        return self._call(self.clear_cli, Trigger.Request(), 'clear_errors')

    def call_calibrate(self):
        return self._call(self.calib_cli, Trigger.Request(), 'calibrate')

    def call_save(self):
        return self._call(self.save_cli, Trigger.Request(), 'save_config')

    def call_estop(self):
        return self._call(self.estop_cli, Trigger.Request(), 'estop')


class ControlGUI(QMainWindow):
    _sig_state     = pyqtSignal()
    _sig_connected = pyqtSignal()
    _sig_calib     = pyqtSignal(str)
    _sig_config    = pyqtSignal(str, float)

    def __init__(self, node: GuiNode):
        super().__init__()
        self.node = node
        self.node.on_state     = self._sig_state.emit
        self.node.on_connected = self._sig_connected.emit
        self.node.on_calib     = self._sig_calib.emit
        self.node.on_config    = self._sig_config.emit

        self._sig_state.connect(self._refresh_status)
        self._sig_connected.connect(self._refresh_connection)
        self._sig_calib.connect(self._on_calib_msg)
        self._sig_config.connect(self._on_config_update)

        self._build_ui()
        self.setWindowTitle('GIM8108-8 Motor Control')
        self.setMinimumWidth(640)

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        v = QVBoxLayout(root)

        v.addWidget(self._make_status_bar())
        v.addWidget(self._make_buttons())
        v.addWidget(self._make_mode_group())
        v.addWidget(self._make_pos_group())
        v.addWidget(self._make_speed_group())
        v.addWidget(self._make_current_group())
        v.addWidget(self._make_pid_group())
        v.addWidget(self._make_calib_log())

    def _make_status_bar(self):
        grp = QGroupBox('Status')
        h = QHBoxLayout(grp)

        self.lbl_conn = QLabel('Disconnected')
        self.lbl_conn.setFont(QFont('', 10, QFont.Bold))
        self.lbl_conn.setStyleSheet('color:red;')
        self.lbl_conn.setMinimumWidth(120)

        self.lbl_pos  = QLabel('Angle:  ---')
        self.lbl_vel  = QLabel('Speed:  ---')
        self.lbl_cur  = QLabel('I:  ---')
        self.lbl_vbus = QLabel('V:  ---')
        for lbl in (self.lbl_pos, self.lbl_vel, self.lbl_cur, self.lbl_vbus):
            lbl.setMinimumWidth(140)
            lbl.setFont(QFont('Monospace', 9))

        for w in (self.lbl_conn, self.lbl_pos, self.lbl_vel, self.lbl_cur, self.lbl_vbus):
            h.addWidget(w)
        h.addStretch()
        return grp

    def _make_buttons(self):
        grp = QGroupBox('Motor Control')
        h = QHBoxLayout(grp)

        self.btn_enable = QPushButton('Enable Motor')
        self.btn_enable.setCheckable(True)
        self.btn_enable.setStyleSheet(
            'QPushButton:checked{background:#2ecc71;color:white;font-weight:bold;}'
        )
        self.btn_enable.toggled.connect(self._on_enable)

        btn_clear = QPushButton('Clear Errors')
        btn_clear.clicked.connect(self._on_clear)

        btn_calib = QPushButton('Calibrate')
        btn_calib.setToolTip('Motor + Encoder offset calibration (~25s)\nDo NOT save yet.')
        btn_calib.clicked.connect(self._on_calibrate)

        btn_save = QPushButton('Save Config')
        btn_save.setToolTip('Save to flash (ODrive will reboot — USB disconnects briefly)')
        btn_save.setStyleSheet('background:#e67e22;color:white;')
        btn_save.clicked.connect(self._on_save)

        btn_estop = QPushButton('E-STOP')
        btn_estop.setStyleSheet('background:#e74c3c;color:white;font-weight:bold;')
        btn_estop.clicked.connect(self._on_estop)

        for w in (self.btn_enable, btn_clear, btn_calib, btn_save, btn_estop):
            h.addWidget(w)
        return grp

    def _make_mode_group(self):
        grp = QGroupBox('Control Mode')
        h = QHBoxLayout(grp)
        self.radio_pos = QRadioButton('Position')
        self.radio_vel = QRadioButton('Velocity')
        self.radio_trq = QRadioButton('Torque')
        self.radio_pos.setChecked(True)
        self.radio_pos.toggled.connect(lambda chk: chk and self._switch_mode(CTRL_POSITION))
        self.radio_vel.toggled.connect(lambda chk: chk and self._switch_mode(CTRL_VELOCITY))
        self.radio_trq.toggled.connect(lambda chk: chk and self._switch_mode(CTRL_TORQUE))
        for w in (self.radio_pos, self.radio_vel, self.radio_trq):
            h.addWidget(w)
        return grp

    def _make_pos_group(self):
        grp = QGroupBox('Target Position (degrees)')
        v = QVBoxLayout(grp)

        h = QHBoxLayout()
        self.slider_pos = QSlider(Qt.Horizontal)
        self.slider_pos.setRange(0, 3600)        # 0.1 deg resolution
        self.slider_pos.setTickPosition(QSlider.TicksBelow)
        self.slider_pos.setTickInterval(900)
        self.slider_pos.valueChanged.connect(self._on_pos_slider)

        self.spin_pos = QDoubleSpinBox()
        self.spin_pos.setRange(0.0, 360.0)
        self.spin_pos.setSingleStep(0.1)
        self.spin_pos.setDecimals(1)
        self.spin_pos.setSuffix(' deg')
        self.spin_pos.setFixedWidth(110)
        self.spin_pos.valueChanged.connect(self._on_pos_spin)

        h.addWidget(self.slider_pos)
        h.addWidget(self.spin_pos)
        v.addLayout(h)

        row = QHBoxLayout()
        for t in ('0', '90', '180', '270', '360 deg'):
            row.addWidget(QLabel(t))
            if 'deg' not in t:
                row.addStretch()
        v.addLayout(row)
        return grp

    def _make_speed_group(self):
        grp = QGroupBox('Trajectory Speed  [motor turns/s]  (position mode)')
        v = QVBoxLayout(grp)

        # ── Velocity limit slider ──────────────────────────────────────────────
        h1 = QHBoxLayout()
        h1.addWidget(QLabel('Vel Limit:'))
        self.slider_spd = QSlider(Qt.Horizontal)
        self.slider_spd.setRange(1, 1000)        # 0.1 turns/s resolution → max 100
        self.slider_spd.setValue(100)            # default 10 turns/s
        self.slider_spd.valueChanged.connect(self._on_spd_slider)
        self.lbl_spd = QLabel('10.0 t/s')
        self.lbl_spd.setFixedWidth(80)
        h1.addWidget(self.slider_spd)
        h1.addWidget(self.lbl_spd)
        v.addLayout(h1)

        row1 = QHBoxLayout()
        for t in ('0.1', '25', '50', '75', '100 t/s'):
            row1.addWidget(QLabel(t))
            if 't/s' not in t:
                row1.addStretch()
        v.addLayout(row1)

        # ── Accel / Decel spinboxes ────────────────────────────────────────────
        h2 = QHBoxLayout()

        h2.addWidget(QLabel('Accel:'))
        self.spin_accel = QDoubleSpinBox()
        self.spin_accel.setRange(0.5, 200.0)
        self.spin_accel.setValue(5.0)
        self.spin_accel.setSingleStep(1.0)
        self.spin_accel.setDecimals(1)
        self.spin_accel.setSuffix(' t/s²')
        self.spin_accel.setFixedWidth(110)
        self.spin_accel.setToolTip('Acceleration limit — higher = faster ramp-up')
        self.spin_accel.valueChanged.connect(self.node.send_traj_accel)
        h2.addWidget(self.spin_accel)

        h2.addSpacing(20)
        h2.addWidget(QLabel('Decel:'))
        self.spin_decel = QDoubleSpinBox()
        self.spin_decel.setRange(0.5, 200.0)
        self.spin_decel.setValue(5.0)
        self.spin_decel.setSingleStep(1.0)
        self.spin_decel.setDecimals(1)
        self.spin_decel.setSuffix(' t/s²')
        self.spin_decel.setFixedWidth(110)
        self.spin_decel.setToolTip('Deceleration limit — higher = stops faster')
        self.spin_decel.valueChanged.connect(self.node.send_traj_decel)
        h2.addWidget(self.spin_decel)

        h2.addStretch()
        v.addLayout(h2)
        return grp

    def _make_current_group(self):
        grp = QGroupBox('Current Limit  [A]')
        v = QVBoxLayout(grp)
        h = QHBoxLayout()

        self.slider_cur = QSlider(Qt.Horizontal)
        self.slider_cur.setRange(1, 250)         # 0.1 A resolution
        self.slider_cur.setValue(70)             # 7 A default
        self.slider_cur.valueChanged.connect(self._on_cur_slider)

        self.lbl_cur_set = QLabel('7.0 A')
        self.lbl_cur_set.setFixedWidth(80)

        h.addWidget(self.slider_cur)
        h.addWidget(self.lbl_cur_set)
        v.addLayout(h)

        row = QHBoxLayout()
        for t in ('0.1 A', '6 A', '13 A', '19 A', '25 A'):
            row.addWidget(QLabel(t))
            if t != '25 A':
                row.addStretch()
        v.addLayout(row)
        return grp

    def _make_pid_group(self):
        grp = QGroupBox('PID Gains  (higher pos_gain/vel_gain = sharper stop, too high = oscillation)')
        grid = QGridLayout(grp)

        def spin(lo, hi, val, step, decimals, suffix):
            w = QDoubleSpinBox()
            w.setRange(lo, hi)
            w.setValue(val)
            w.setSingleStep(step)
            w.setDecimals(decimals)
            w.setSuffix(suffix)
            w.setFixedWidth(130)
            return w

        grid.addWidget(QLabel('Pos Gain:'), 0, 0)
        self.spin_pos_gain = spin(0, 200, 20.0, 1.0, 1, ' (t/s)/t')
        self.spin_pos_gain.setToolTip('Position P gain — increase for stiffer hold')
        self.spin_pos_gain.valueChanged.connect(self.node.send_pos_gain)
        grid.addWidget(self.spin_pos_gain, 0, 1)

        grid.addWidget(QLabel('Vel Gain:'), 0, 2)
        self.spin_vel_gain = spin(0, 5.0, 0.16, 0.01, 3, ' A/(t/s)')
        self.spin_vel_gain.setToolTip('Velocity P gain — increase to damp oscillation')
        self.spin_vel_gain.valueChanged.connect(self.node.send_vel_gain)
        grid.addWidget(self.spin_vel_gain, 0, 3)

        grid.addWidget(QLabel('Vel Int Gain:'), 0, 4)
        self.spin_vel_int = spin(0, 10.0, 0.32, 0.01, 3, ' A/(t/s²)')
        self.spin_vel_int.setToolTip('Velocity I gain — eliminates steady-state error')
        self.spin_vel_int.valueChanged.connect(self.node.send_vel_int_gain)
        grid.addWidget(self.spin_vel_int, 0, 5)

        btn_apply = QPushButton('Apply')
        btn_apply.setToolTip('Re-send current PID values to the motor')
        btn_apply.clicked.connect(self._apply_pid)
        grid.addWidget(btn_apply, 0, 6)

        return grp

    def _apply_pid(self):
        self.node.send_pos_gain(self.spin_pos_gain.value())
        self.node.send_vel_gain(self.spin_vel_gain.value())
        self.node.send_vel_int_gain(self.spin_vel_int.value())

    def _make_calib_log(self):
        grp = QGroupBox('Calibration Log')
        v = QVBoxLayout(grp)
        self.calib_log = QTextEdit()
        self.calib_log.setReadOnly(True)
        self.calib_log.setMaximumHeight(90)
        self.calib_log.setFont(QFont('Monospace', 9))
        self.calib_log.setPlaceholderText('Calibration messages appear here...')
        v.addWidget(self.calib_log)
        return grp

    # ── Button handlers ───────────────────────────────────────────────────────

    def _on_enable(self, checked: bool):
        err = self.node.call_enable(checked)
        if err:
            QMessageBox.warning(self, 'Warning', err)
            self.btn_enable.blockSignals(True)
            self.btn_enable.setChecked(not checked)
            self.btn_enable.blockSignals(False)
            return
        self.btn_enable.setText('Disable Motor' if checked else 'Enable Motor')

    def _on_clear(self):
        err = self.node.call_clear()
        if err:
            QMessageBox.warning(self, 'Warning', err)

    def _on_calibrate(self):
        err = self.node.call_calibrate()
        if err:
            QMessageBox.warning(self, 'Warning', err)
            return
        self.calib_log.clear()
        self.calib_log.append('Calibration started...')

    def _on_save(self):
        reply = QMessageBox.question(
            self, 'Save Configuration',
            'ODrive will REBOOT after saving.\n'
            'USB will disconnect briefly (~5s) then reconnect.\n\n'
            'Continue?',
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            err = self.node.call_save()
            if err:
                QMessageBox.warning(self, 'Warning', err)
            else:
                self.calib_log.append('Saving config — ODrive rebooting...')

    def _on_estop(self):
        err = self.node.call_estop()
        if err:
            QMessageBox.warning(self, 'E-STOP', err)
        self.btn_enable.blockSignals(True)
        self.btn_enable.setChecked(False)
        self.btn_enable.setText('Enable Motor')
        self.btn_enable.blockSignals(False)

    def _switch_mode(self, mode: int):
        self.node.send_mode(mode)
        self.slider_pos.setEnabled(mode == CTRL_POSITION)
        self.spin_pos.setEnabled(mode == CTRL_POSITION)
        self.slider_spd.setEnabled(mode != CTRL_TORQUE)

    # ── Slider callbacks ──────────────────────────────────────────────────────

    def _on_pos_slider(self, val: int):
        deg = val / 10.0
        self.spin_pos.blockSignals(True)
        self.spin_pos.setValue(deg)
        self.spin_pos.blockSignals(False)
        if self.radio_pos.isChecked():
            self.node.send_pos_deg(deg)

    def _on_pos_spin(self, val: float):
        self.slider_pos.blockSignals(True)
        self.slider_pos.setValue(int(val * 10))
        self.slider_pos.blockSignals(False)
        if self.radio_pos.isChecked():
            self.node.send_pos_deg(val)

    def _on_spd_slider(self, val: int):
        turns_s = val / 10.0
        self.lbl_spd.setText(f'{turns_s:.1f} t/s')
        if self.radio_pos.isChecked():
            self.node.send_traj_vel(turns_s)
        elif self.radio_vel.isChecked():
            self.node.send_vel(turns_s)

    def _on_cur_slider(self, val: int):
        a = val / 10.0
        self.lbl_cur_set.setText(f'{a:.1f} A')
        self.node.send_cur_limit(a)

    # ── Signal handlers ───────────────────────────────────────────────────────

    def _refresh_status(self):
        if not self.node.is_connected:
            return  # keep "---" shown by _refresh_connection

        pos = self.node.pos_deg
        # At the 360°/0° boundary, snap to 0.0 to avoid -0.x display
        if pos >= 360.0:
            pos -= 360.0
        self.lbl_pos.setText(f'Angle:  {pos:6.1f} °')

        vel = self.node.vel_out_ts
        self.lbl_vel.setText(f'Speed:  {vel:5.2f} t/s')

        self.lbl_cur.setText(f'I:  {self.node.current_a:5.2f} A')

        vbus = self.node.voltage_v
        if vbus is None:
            self.lbl_vbus.setText('V:   ---')
            self.lbl_vbus.setStyleSheet('')
        else:
            color = 'red' if vbus < 10.0 else ('darkorange' if vbus < 14.0 else 'black')
            self.lbl_vbus.setText(f'V:  {vbus:5.1f} V')
            self.lbl_vbus.setStyleSheet(f'color:{color};')

    def _refresh_connection(self):
        if self.node.is_connected:
            self.lbl_conn.setText('Connected')
            self.lbl_conn.setStyleSheet('color:green;font-weight:bold;')
        else:
            self.lbl_conn.setText('Disconnected')
            self.lbl_conn.setStyleSheet('color:red;font-weight:bold;')
            # Clear readings when disconnected
            self.lbl_pos.setText('Angle:  ---')
            self.lbl_vel.setText('Speed:  ---')
            self.lbl_cur.setText('I:  ---')
            self.lbl_vbus.setText('V:  ---')
            self.lbl_vbus.setStyleSheet('')

    def _on_calib_msg(self, msg: str):
        self.calib_log.append(msg)

    def _on_config_update(self, key: str, value: float):
        """Update UI spinboxes/sliders when ODrive reports its actual config."""
        def _set(widget, v):
            widget.blockSignals(True)
            widget.setValue(v)
            widget.blockSignals(False)

        if key == 'pos_gain':
            _set(self.spin_pos_gain, value)
        elif key == 'vel_gain':
            _set(self.spin_vel_gain, value)
        elif key == 'vel_int':
            _set(self.spin_vel_int, value)
        elif key == 'traj_vel':
            _set(self.slider_spd, int(round(value * 10)))
        elif key == 'traj_accel':
            _set(self.spin_accel, value)
        elif key == 'traj_decel':
            _set(self.spin_decel, value)


def main(args=None):
    rclpy.init(args=args)
    ros_node = GuiNode()

    app = QApplication(sys.argv)

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    window = ControlGUI(ros_node)
    window.show()
    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
