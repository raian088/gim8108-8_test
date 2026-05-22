"""
GIM8108-8 Unified Motor Control & Motion GUI
Tab 1: Motor Control  (manual control, PID, limits)
Tab 2: Motion         (record & playback)
"""

import json
import math
import sys
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

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

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget,
    QListWidgetItem, QMainWindow, QMessageBox, QProgressBar, QPushButton,
    QRadioButton, QSlider, QTabWidget, QTextEdit, QVBoxLayout, QWidget,
)

CTRL_TORQUE   = 1
CTRL_VELOCITY = 2
CTRL_POSITION = 3

DRIVER_NS  = '/gim8108_motor_node'
MOTION_DIR = Path.home() / 'gim8108_motions'


# ── Motion data model ─────────────────────────────────────────────────────────

@dataclass
class Motion:
    name: str
    hz: float
    frames: list = field(default_factory=list)

    @property
    def duration(self) -> float:
        return len(self.frames) / self.hz if self.hz > 0 else 0.0

    def save(self, path: Path):
        path.write_text(json.dumps(
            {'name': self.name, 'hz': self.hz, 'frames': self.frames}, indent=2
        ))

    @classmethod
    def load(cls, path: Path) -> 'Motion':
        d = json.loads(path.read_text())
        return cls(name=d['name'], hz=float(d['hz']), frames=list(d['frames']))


# ── ROS node ──────────────────────────────────────────────────────────────────

class GuiNode(Node):
    def __init__(self):
        super().__init__('gim8108_gui')

        # Publishers
        self.pub_pos          = self.create_publisher(Float64, f'{DRIVER_NS}/cmd_pos_deg',        10)
        self.pub_vel          = self.create_publisher(Float64, f'{DRIVER_NS}/cmd_vel',             10)
        self.pub_torque       = self.create_publisher(Float64, f'{DRIVER_NS}/cmd_torque',          10)
        self.pub_vel_lim      = self.create_publisher(Float64, f'{DRIVER_NS}/vel_limit',           10)
        self.pub_cur_lim      = self.create_publisher(Float64, f'{DRIVER_NS}/current_limit',       10)
        self.pub_traj_vel     = self.create_publisher(Float64, f'{DRIVER_NS}/traj_vel_limit',      10)
        self.pub_mode         = self.create_publisher(Int32,   f'{DRIVER_NS}/control_mode',        10)
        self.pub_pos_gain     = self.create_publisher(Float64, f'{DRIVER_NS}/pos_gain',            10)
        self.pub_vel_gain     = self.create_publisher(Float64, f'{DRIVER_NS}/vel_gain',            10)
        self.pub_vel_int_gain = self.create_publisher(Float64, f'{DRIVER_NS}/vel_integrator_gain', 10)
        self.pub_traj_accel   = self.create_publisher(Float64, f'{DRIVER_NS}/traj_accel_limit',    10)
        self.pub_traj_decel   = self.create_publisher(Float64, f'{DRIVER_NS}/traj_decel_limit',    10)

        # Subscriptions
        self.create_subscription(JointState, f'{DRIVER_NS}/joint_state',  self._on_state,        10)
        self.create_subscription(Bool,       f'{DRIVER_NS}/is_connected', self._on_connected,    _QOS_LATCHED)
        self.create_subscription(String,     f'{DRIVER_NS}/calib_status', self._on_calib_status, 10)
        self.create_subscription(Float64,    f'{DRIVER_NS}/bus_voltage',  self._on_voltage,      10)
        ns = DRIVER_NS
        self.create_subscription(Float64, f'{ns}/config/pos_gain',            self._on_cfg('pos_gain'),   _QOS_LATCHED)
        self.create_subscription(Float64, f'{ns}/config/vel_gain',            self._on_cfg('vel_gain'),   _QOS_LATCHED)
        self.create_subscription(Float64, f'{ns}/config/vel_integrator_gain', self._on_cfg('vel_int'),    _QOS_LATCHED)
        self.create_subscription(Float64, f'{ns}/config/traj_vel_limit',      self._on_cfg('traj_vel'),   _QOS_LATCHED)
        self.create_subscription(Float64, f'{ns}/config/traj_accel_limit',    self._on_cfg('traj_accel'), _QOS_LATCHED)
        self.create_subscription(Float64, f'{ns}/config/traj_decel_limit',    self._on_cfg('traj_decel'), _QOS_LATCHED)
        self.create_subscription(Float64, f'{ns}/config/vel_limit',           self._on_cfg('vel_limit'),  _QOS_LATCHED)
        self.create_subscription(Float64, f'{ns}/config/current_lim',         self._on_cfg('cur_limit'),  _QOS_LATCHED)

        # Service clients
        self.enable_cli = self.create_client(SetBool, f'{DRIVER_NS}/enable')
        self.clear_cli  = self.create_client(Trigger, f'{DRIVER_NS}/clear_errors')
        self.calib_cli  = self.create_client(Trigger, f'{DRIVER_NS}/calibrate')
        self.save_cli   = self.create_client(Trigger, f'{DRIVER_NS}/save_config')
        self.estop_cli  = self.create_client(Trigger, f'{DRIVER_NS}/estop')

        # State
        self.pos_deg      = 0.0
        self.pos_abs_deg  = 0.0   # modulo なしの絶対角度 (deg)
        self.vel_out_ts   = 0.0
        self.current_a    = 0.0
        self.voltage_v    = None
        self.is_connected = False
        self._cfg         = {}

        self.on_state     = None
        self.on_connected = None
        self.on_calib     = None
        self.on_config    = None

    def _on_state(self, msg: JointState):
        if msg.position:
            self.pos_abs_deg = math.degrees(msg.position[0])
            self.pos_deg = self.pos_abs_deg % 360.0
        if msg.velocity:
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
            if self.on_config: self.on_config(key, msg.data)
        return _cb

    def _on_connected(self, msg: Bool):
        if self.is_connected != msg.data:
            self.is_connected = msg.data
            if self.on_connected: self.on_connected()

    def _on_calib_status(self, msg: String):
        if self.on_calib: self.on_calib(msg.data)

    def _call(self, client, request, label: str):
        if not client.service_is_ready():
            return f'{label}: driver not ready'
        client.call_async(request)
        return None

    # Publish helpers
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


# ── GUI ───────────────────────────────────────────────────────────────────────

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

        # Motion state
        self._recorded: list = []
        self._record_hz: float = 10.0
        self._record_timer = QTimer()
        self._record_timer.timeout.connect(self._record_frame)
        self._motion: Optional[Motion] = None
        self._play_idx: int = 0
        self._play_timer = QTimer()
        self._play_timer.timeout.connect(self._play_step)

        MOTION_DIR.mkdir(exist_ok=True)
        self._build_ui()
        self.setWindowTitle('GIM8108-8 Motor Control')
        self.setMinimumWidth(760)
        self._refresh_motion_list()

    # ── UI layout ─────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        v = QVBoxLayout(root)
        v.addWidget(self._make_status_bar())
        tabs = QTabWidget()
        tabs.addTab(self._make_control_tab(), 'Motor Control')
        tabs.addTab(self._make_motion_tab(),  'Motion')
        v.addWidget(tabs)

    # ── Shared status bar ─────────────────────────────────────────────────────

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

    # ── Motor Control tab ──────────────────────────────────────────────────────

    def _make_control_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.addWidget(self._make_buttons())
        v.addWidget(self._make_mode_group())
        v.addWidget(self._make_pos_group())
        v.addWidget(self._make_speed_group())
        v.addWidget(self._make_current_group())
        v.addWidget(self._make_pid_group())
        v.addWidget(self._make_calib_log())
        return w

    def _make_buttons(self):
        grp = QGroupBox('Motor Control')
        v = QVBoxLayout(grp)

        h1 = QHBoxLayout()
        self.btn_enable = QPushButton('Enable Motor')
        self.btn_enable.setCheckable(True)
        self.btn_enable.setStyleSheet(
            'QPushButton:checked{background:#2ecc71;color:white;font-weight:bold;}'
        )
        self.btn_enable.toggled.connect(self._on_enable)

        self.btn_free_ctrl = QPushButton('Free (Torque OFF)')
        self.btn_free_ctrl.setCheckable(True)
        self.btn_free_ctrl.setStyleSheet(
            'QPushButton:checked{background:#e67e22;color:white;font-weight:bold;}'
        )
        self.btn_free_ctrl.toggled.connect(self._on_free_ctrl_toggled)

        btn_home = QPushButton('→ Origin (0°)')
        btn_home.setToolTip('Enable motor and move to 0°')
        btn_home.setStyleSheet('background:#2980b9;color:white;font-weight:bold;')
        btn_home.clicked.connect(self._on_home)

        btn_clear = QPushButton('Clear Errors')
        btn_clear.clicked.connect(self._on_clear)

        btn_estop = QPushButton('E-STOP')
        btn_estop.setStyleSheet('background:#e74c3c;color:white;font-weight:bold;')
        btn_estop.clicked.connect(self._on_estop)

        for w in (self.btn_enable, self.btn_free_ctrl, btn_home, btn_clear, btn_estop):
            h1.addWidget(w)
        v.addLayout(h1)

        h2 = QHBoxLayout()
        btn_calib = QPushButton('Calibrate')
        btn_calib.setToolTip('Motor + Encoder offset calibration (~25s)')
        btn_calib.clicked.connect(self._on_calibrate)
        btn_save = QPushButton('Save Config')
        btn_save.setToolTip('Save to flash (ODrive will reboot)')
        btn_save.setStyleSheet('background:#e67e22;color:white;')
        btn_save.clicked.connect(self._on_save)
        for w in (btn_calib, btn_save):
            h2.addWidget(w)
        h2.addStretch()
        v.addLayout(h2)

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
        self.slider_pos.setRange(0, 3600)
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
        h1 = QHBoxLayout()
        h1.addWidget(QLabel('Vel Limit:'))
        self.slider_spd = QSlider(Qt.Horizontal)
        self.slider_spd.setRange(1, 1000)
        self.slider_spd.setValue(100)
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
        h2 = QHBoxLayout()
        h2.addWidget(QLabel('Accel:'))
        self.spin_accel = QDoubleSpinBox()
        self.spin_accel.setRange(0.5, 200.0)
        self.spin_accel.setValue(5.0)
        self.spin_accel.setSingleStep(1.0)
        self.spin_accel.setDecimals(1)
        self.spin_accel.setSuffix(' t/s²')
        self.spin_accel.setFixedWidth(110)
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
        self.slider_cur.setRange(1, 250)
        self.slider_cur.setValue(70)
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
            w.setRange(lo, hi); w.setValue(val); w.setSingleStep(step)
            w.setDecimals(decimals); w.setSuffix(suffix); w.setFixedWidth(130)
            return w

        grid.addWidget(QLabel('Pos Gain:'), 0, 0)
        self.spin_pos_gain = spin(0, 200, 20.0, 1.0, 1, ' (t/s)/t')
        self.spin_pos_gain.valueChanged.connect(self.node.send_pos_gain)
        grid.addWidget(self.spin_pos_gain, 0, 1)

        grid.addWidget(QLabel('Vel Gain:'), 0, 2)
        self.spin_vel_gain = spin(0, 5.0, 0.16, 0.01, 3, ' A/(t/s)')
        self.spin_vel_gain.valueChanged.connect(self.node.send_vel_gain)
        grid.addWidget(self.spin_vel_gain, 0, 3)

        grid.addWidget(QLabel('Vel Int Gain:'), 0, 4)
        self.spin_vel_int = spin(0, 10.0, 0.32, 0.01, 3, ' A/(t/s²)')
        self.spin_vel_int.valueChanged.connect(self.node.send_vel_int_gain)
        grid.addWidget(self.spin_vel_int, 0, 5)

        btn_apply = QPushButton('Apply')
        btn_apply.clicked.connect(self._apply_pid)
        grid.addWidget(btn_apply, 0, 6)
        return grp

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

    # ── Motion tab ────────────────────────────────────────────────────────────

    def _make_motion_tab(self):
        w = QWidget()
        h = QHBoxLayout(w)
        h.addWidget(self._make_record_panel(), 1)
        h.addWidget(self._make_play_panel(), 1)
        return w

    def _make_record_panel(self):
        grp = QGroupBox('Record')
        v = QVBoxLayout(grp)

        hFree = QHBoxLayout()
        self.btn_free_rec = QPushButton('Free (Torque OFF)')
        self.btn_free_rec.setCheckable(True)
        self.btn_free_rec.setStyleSheet(
            'QPushButton:checked{background:#e67e22;color:white;font-weight:bold;}'
        )
        self.btn_free_rec.toggled.connect(self._on_free_rec_toggled)
        hFree.addWidget(self.btn_free_rec)
        hFree.addStretch()
        v.addLayout(hFree)

        hHz = QHBoxLayout()
        hHz.addWidget(QLabel('Capture Hz:'))
        self.cmb_hz = QComboBox()
        for hz in (5, 10, 20, 30):
            self.cmb_hz.addItem(f'{hz} Hz', float(hz))
        self.cmb_hz.setCurrentIndex(1)
        hHz.addWidget(self.cmb_hz)
        hHz.addStretch()
        v.addLayout(hHz)

        self.lbl_frames = QLabel('Frames: 0  (0.0 s)')
        v.addWidget(self.lbl_frames)

        hBtn = QHBoxLayout()
        self.btn_rec = QPushButton('● Record')
        self.btn_rec.setStyleSheet('background:#e74c3c;color:white;font-weight:bold;')
        self.btn_rec.clicked.connect(self._start_record)
        self.btn_stop_rec = QPushButton('■ Stop')
        self.btn_stop_rec.setEnabled(False)
        self.btn_stop_rec.clicked.connect(self._stop_record)
        hBtn.addWidget(self.btn_rec)
        hBtn.addWidget(self.btn_stop_rec)
        v.addLayout(hBtn)

        v.addWidget(QLabel('Motion name:'))
        self.edit_name = QLineEdit('motion_01')
        v.addWidget(self.edit_name)

        btn_save_motion = QPushButton('Save Motion')
        btn_save_motion.clicked.connect(self._save_motion)
        v.addWidget(btn_save_motion)

        v.addStretch()
        return grp

    def _make_play_panel(self):
        grp = QGroupBox('Playback')
        v = QVBoxLayout(grp)

        v.addWidget(QLabel('Saved motions  (~/gim8108_motions/):'))
        self.lst = QListWidget()
        self.lst.setMaximumHeight(120)
        self.lst.itemSelectionChanged.connect(self._on_motion_selected)
        v.addWidget(self.lst)

        hListBtn = QHBoxLayout()
        btn_refresh = QPushButton('Refresh')
        btn_refresh.clicked.connect(self._refresh_motion_list)
        btn_load = QPushButton('Load file…')
        btn_load.clicked.connect(self._load_file)
        btn_del = QPushButton('Delete')
        btn_del.clicked.connect(self._delete_motion)
        for b in (btn_refresh, btn_load, btn_del):
            hListBtn.addWidget(b)
        v.addLayout(hListBtn)

        self.lbl_info = QLabel('No motion loaded')
        self.lbl_info.setFont(QFont('Monospace', 9))
        v.addWidget(self.lbl_info)

        self.progress = QProgressBar()
        self.progress.setFormat('%v / %m  frames')
        v.addWidget(self.progress)

        hSpd = QHBoxLayout()
        hSpd.addWidget(QLabel('Speed:'))
        self.cmb_speed = QComboBox()
        for label, val in (('0.25x', 0.25), ('0.5x', 0.5),
                           ('1x', 1.0), ('2x', 2.0), ('4x', 4.0)):
            self.cmb_speed.addItem(label, val)
        self.cmb_speed.setCurrentIndex(2)
        hSpd.addWidget(self.cmb_speed)
        self.chk_loop = QCheckBox('Loop')
        hSpd.addWidget(self.chk_loop)
        hSpd.addStretch()
        v.addLayout(hSpd)

        hPlay = QHBoxLayout()
        self.btn_play = QPushButton('▶  Play')
        self.btn_play.setStyleSheet('background:#2ecc71;color:white;font-weight:bold;')
        self.btn_play.setEnabled(False)
        self.btn_play.clicked.connect(self._start_play)
        self.btn_stop_play = QPushButton('■  Stop')
        self.btn_stop_play.setEnabled(False)
        self.btn_stop_play.clicked.connect(self._stop_play)
        hPlay.addWidget(self.btn_play)
        hPlay.addWidget(self.btn_stop_play)
        v.addLayout(hPlay)

        v.addStretch()
        return grp

    # ── Motor Control handlers ─────────────────────────────────────────────────

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
            'USB will disconnect briefly (~5s) then reconnect.\n\nContinue?',
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

    def _apply_pid(self):
        self.node.send_pos_gain(self.spin_pos_gain.value())
        self.node.send_vel_gain(self.spin_vel_gain.value())
        self.node.send_vel_int_gain(self.spin_vel_int.value())

    # ── Motion handlers ────────────────────────────────────────────────────────

    def _set_free(self, checked: bool):
        self.node.call_enable(not checked)
        label = 'Free (Torque OFF)  ← 手で動かせます' if checked else 'Free (Torque OFF)'
        for btn in (self.btn_free_ctrl, self.btn_free_rec):
            btn.blockSignals(True)
            btn.setChecked(checked)
            btn.setText(label)
            btn.blockSignals(False)

    def _on_free_ctrl_toggled(self, checked: bool):
        self._set_free(checked)

    def _on_free_rec_toggled(self, checked: bool):
        self._set_free(checked)

    def _on_home(self):
        self._set_free(False)
        self.btn_enable.blockSignals(True)
        self.btn_enable.setChecked(True)
        self.btn_enable.setText('Disable Motor')
        self.btn_enable.blockSignals(False)
        self.node.call_enable(True)
        # 現在の絶対角度から最も近い 0° (360° の倍数) を計算
        frac = self.node.pos_abs_deg % 360.0
        if frac <= 180.0:
            target = self.node.pos_abs_deg - frac          # 手前の 0° へ戻る
        else:
            target = self.node.pos_abs_deg + (360.0 - frac)  # 次の 0° へ進む
        QTimer.singleShot(300, lambda: self.node.send_pos_deg(target))

    def _start_record(self):
        self._recorded = []
        self._record_hz = self.cmb_hz.currentData()
        self._record_timer.start(int(1000 / self._record_hz))
        self.btn_rec.setEnabled(False)
        self.btn_stop_rec.setEnabled(True)
        self.lbl_frames.setText('Recording…  0 frames')

    def _record_frame(self):
        self._recorded.append(self.node.pos_deg)
        n = len(self._recorded)
        self.lbl_frames.setText(f'Recording…  {n} frames  ({n / self._record_hz:.1f} s)')

    def _stop_record(self):
        self._record_timer.stop()
        self.btn_rec.setEnabled(True)
        self.btn_stop_rec.setEnabled(False)
        self._set_free(False)
        self.node.call_enable(True)
        n = len(self._recorded)
        self.lbl_frames.setText(
            f'Done: {n} frames  ({n / self._record_hz:.1f} s)  — press Save to keep'
        )

    def _save_motion(self):
        if not self._recorded:
            QMessageBox.warning(self, 'No Data', 'Record a motion first.')
            return
        raw = self.edit_name.text().strip() or 'motion'
        name = ''.join(c if c.isalnum() or c in '-_' else '_' for c in raw)
        path = MOTION_DIR / f'{name}.json'
        if path.exists():
            if QMessageBox.question(self, 'Overwrite?',
                    f'"{name}.json" already exists. Overwrite?',
                    QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
                return
        Motion(name=name, hz=self._record_hz, frames=list(self._recorded)).save(path)
        self._refresh_motion_list()
        QMessageBox.information(self, 'Saved', f'Saved:\n{path}')

    def _refresh_motion_list(self):
        self.lst.clear()
        for p in sorted(MOTION_DIR.glob('*.json')):
            self.lst.addItem(QListWidgetItem(p.stem))

    def _load_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, 'Open motion file', str(MOTION_DIR), 'JSON (*.json)')
        if path:
            self._load_motion(Path(path))

    def _on_motion_selected(self):
        items = self.lst.selectedItems()
        if not items:
            return
        self._load_motion(MOTION_DIR / f'{items[0].text()}.json')

    def _load_motion(self, path: Path):
        try:
            m = Motion.load(path)
        except Exception as e:
            QMessageBox.warning(self, 'Load Error', str(e))
            return
        self._motion = m
        n = len(m.frames)
        self.lbl_info.setText(f'{m.name}   {n} frames @ {m.hz} Hz   ({m.duration:.1f} s)')
        self.progress.setMaximum(max(n, 1))
        self.progress.setValue(0)
        self.btn_play.setEnabled(True)

    def _delete_motion(self):
        items = self.lst.selectedItems()
        if not items:
            return
        name = items[0].text()
        if QMessageBox.question(self, 'Delete', f'Delete "{name}"?',
                QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            (MOTION_DIR / f'{name}.json').unlink(missing_ok=True)
            self._refresh_motion_list()
            if self._motion and self._motion.name == name:
                self._motion = None
                self.lbl_info.setText('No motion loaded')
                self.btn_play.setEnabled(False)

    def _start_play(self):
        if not self._motion:
            return
        if not self.node.is_connected:
            QMessageBox.warning(self, 'Not Connected', 'Motor driver is not connected.')
            return
        self.node.call_enable(True)
        self._play_idx = 0
        speed = self.cmb_speed.currentData()
        interval = max(1, int(1000 / (self._motion.hz * speed)))
        self._play_timer.start(interval)
        self.btn_play.setEnabled(False)
        self.btn_stop_play.setEnabled(True)

    def _play_step(self):
        if not self._motion:
            self._stop_play()
            return
        if self._play_idx >= len(self._motion.frames):
            if self.chk_loop.isChecked():
                self._play_idx = 0
            else:
                self._stop_play()
                return
        self.node.send_pos_deg(self._motion.frames[self._play_idx])
        self.progress.setValue(self._play_idx + 1)
        self._play_idx += 1

    def _stop_play(self):
        self._play_timer.stop()
        self.btn_play.setEnabled(self._motion is not None)
        self.btn_stop_play.setEnabled(False)

    # ── Status refresh ─────────────────────────────────────────────────────────

    def _refresh_status(self):
        if not self.node.is_connected:
            return
        pos = self.node.pos_deg
        if pos >= 360.0:
            pos -= 360.0
        self.lbl_pos.setText(f'Angle:  {pos:6.1f} °')
        self.lbl_vel.setText(f'Speed:  {self.node.vel_out_ts:5.2f} t/s')
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
            for lbl in (self.lbl_pos, self.lbl_vel, self.lbl_cur, self.lbl_vbus):
                lbl.setText(lbl.text().split(':')[0] + ':  ---')
            self.lbl_vbus.setStyleSheet('')
            self._stop_play()

    def _on_calib_msg(self, msg: str):
        self.calib_log.append(msg)

    def _on_config_update(self, key: str, value: float):
        def _set(widget, v):
            widget.blockSignals(True)
            widget.setValue(v)
            widget.blockSignals(False)

        if key == 'pos_gain':     _set(self.spin_pos_gain, value)
        elif key == 'vel_gain':   _set(self.spin_vel_gain, value)
        elif key == 'vel_int':    _set(self.spin_vel_int,  value)
        elif key == 'traj_vel':   _set(self.slider_spd, int(round(value * 10)))
        elif key == 'traj_accel': _set(self.spin_accel, value)
        elif key == 'traj_decel': _set(self.spin_decel, value)
        elif key == 'cur_limit':
            _set(self.slider_cur, int(round(value * 10)))
            self.lbl_cur_set.setText(f'{value:.1f} A')


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()

    app = QApplication(sys.argv)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    window = ControlGUI(node)
    window.show()
    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
