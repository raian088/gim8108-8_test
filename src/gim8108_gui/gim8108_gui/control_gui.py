"""
GIM8108-8 Multi-Motor Control GUI
Left  : motor list (status cards + Add)
Right : selected motor — Motor Control tab | Motion tab
"""

import collections
import json
import math
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

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
    QApplication, QCheckBox, QComboBox, QDialog, QDialogButtonBox,
    QDoubleSpinBox, QFileDialog, QFormLayout, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem, QMainWindow,
    QMessageBox, QProgressBar, QPushButton, QRadioButton, QScrollArea,
    QSlider, QSplitter, QTabWidget, QTextEdit, QVBoxLayout, QWidget,
)

# ── Constants ─────────────────────────────────────────────────────────────────

CTRL_TORQUE   = 1
CTRL_VELOCITY = 2
CTRL_POSITION = 3

MOTION_DIR   = Path.home() / 'gim8108_motions'
SEQ_DIR      = Path.home() / 'gim8108_sequences'
DEFAULT_NS   = '/gim8108_motor_node'   # backward-compatible with start.launch.py
MOTORS_CONFIG = Path.home() / '.config' / 'gim8108' / 'motors.json'
PANEL_CONFIG  = Path.home() / '.config' / 'gim8108' / 'panel_settings.json'

# ── Dark stylesheet ───────────────────────────────────────────────────────────

DARK_STYLE = """
QMainWindow, QDialog { background:#1a1a1a; }
QWidget { background:#1a1a1a; color:#d0d0d0; font-size:11px; }

QGroupBox {
    border:1px solid #3a3a3a; border-radius:5px;
    margin-top:10px; padding:8px 6px 6px 6px;
    color:#777777; font-size:10px; font-weight:bold;
    letter-spacing:1px;
}
QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 6px; }

QPushButton {
    background:#252525; color:#b0b0b0;
    border:1px solid #3d3d3d; border-radius:4px;
    padding:6px 14px; min-height:26px; font-size:11px;
}
QPushButton:hover   { background:#303030; border-color:#5a5a5a; color:#e0e0e0; }
QPushButton:pressed { background:#1a1a1a; }
QPushButton:checked { background:#505050; color:#ffffff; border-color:#888; font-weight:bold; }
QPushButton:disabled { color:#404040; border-color:#2a2a2a; }

QTabWidget::pane { border:1px solid #3a3a3a; background:#1a1a1a; top:-1px; }
QTabBar::tab {
    background:#222222; color:#666666; border:1px solid #3a3a3a;
    border-bottom:none; padding:7px 22px; margin-right:2px;
    border-radius:4px 4px 0 0;
}
QTabBar::tab:selected { background:#1a1a1a; color:#e0e0e0; border-bottom:1px solid #1a1a1a; }
QTabBar::tab:hover    { background:#2a2a2a; color:#bbbbbb; }

QSlider::groove:horizontal { height:3px; background:#2e2e2e; border-radius:2px; }
QSlider::sub-page:horizontal { background:#666666; border-radius:2px; }
QSlider::handle:horizontal {
    background:#aaaaaa; width:13px; height:13px;
    border-radius:7px; margin:-5px 0; border:1px solid #888;
}
QSlider::handle:horizontal:hover { background:#cccccc; }

QDoubleSpinBox, QSpinBox, QLineEdit {
    background:#242424; color:#d0d0d0; border:1px solid #3d3d3d;
    border-radius:3px; padding:3px 6px; selection-background-color:#505050;
}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button,
QSpinBox::up-button,       QSpinBox::down-button {
    background:#2e2e2e; border:none; width:16px;
}
QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover,
QSpinBox::up-button:hover,       QSpinBox::down-button:hover { background:#404040; }

QComboBox {
    background:#242424; color:#d0d0d0; border:1px solid #3d3d3d;
    border-radius:3px; padding:3px 8px; min-height:22px;
}
QComboBox::drop-down { border:none; width:20px; }
QComboBox QAbstractItemView {
    background:#242424; color:#d0d0d0; border:1px solid #3d3d3d;
    selection-background-color:#505050;
}

QListWidget {
    background:#1e1e1e; color:#cccccc; border:1px solid #3a3a3a;
    border-radius:3px; outline:none;
}
QListWidget::item         { padding:4px 8px; border-radius:2px; }
QListWidget::item:selected { background:#484848; color:#ffffff; }
QListWidget::item:hover    { background:#2d2d2d; }

QTextEdit {
    background:#141414; color:#999999; border:1px solid #3a3a3a;
    border-radius:3px; selection-background-color:#505050;
}
QProgressBar {
    background:#222222; border:1px solid #3a3a3a; border-radius:3px;
    text-align:center; color:#999999; font-size:10px;
}
QProgressBar::chunk { background:#555555; border-radius:2px; }

QRadioButton { color:#b0b0b0; spacing:6px; }
QRadioButton::indicator {
    width:14px; height:14px; border-radius:7px;
    border:2px solid #4a4a4a; background:#222;
}
QRadioButton::indicator:checked { background:#888; border-color:#aaa; }

QCheckBox { color:#b0b0b0; spacing:6px; }
QCheckBox::indicator {
    width:14px; height:14px; border-radius:3px;
    border:2px solid #4a4a4a; background:#222;
}
QCheckBox::indicator:checked { background:#888; border-color:#aaa; }

QLabel    { color:#b0b0b0; background:transparent; }
QSplitter { background:#1a1a1a; }
QSplitter::handle { background:#2e2e2e; width:2px; }

QScrollBar:vertical   { background:#1e1e1e; width:7px; margin:0; }
QScrollBar:horizontal { background:#1e1e1e; height:7px; margin:0; }
QScrollBar::handle:vertical   { background:#404040; border-radius:3px; min-height:20px; }
QScrollBar::handle:horizontal { background:#404040; border-radius:3px; min-width:20px;  }
QScrollBar::handle:vertical:hover,
QScrollBar::handle:horizontal:hover { background:#606060; }
QScrollBar::add-line, QScrollBar::sub-line { height:0; width:0; }

QFrame[frameShape="4"], QFrame[frameShape="5"] { color:#333333; }
"""

# ── Real-time status graph ────────────────────────────────────────────────────

GRAPH_MAXLEN = 300   # samples retained (~60 s at 5 Hz refresh)

_GRAPH_PLOTS = [
    ('Angle',    '°',     '#7799ff'),
    ('Velocity', 't/s',   '#66cc88'),
    ('Current',  'A',     '#ffaa55'),
    ('Temp',     '°C',    '#ff6666'),
    ('Voltage',  'V',     '#aaaaaa'),
]


class StatusGraphCanvas(FigureCanvasQTAgg):
    def __init__(self):
        self._fig = Figure(figsize=(8, 6), facecolor='#1a1a1a')
        super().__init__(self._fig)

        n = len(_GRAPH_PLOTS)
        self._axes = self._fig.subplots(n, 1, sharex=True)
        self._lines = []
        self._buf_t: collections.deque = collections.deque(maxlen=GRAPH_MAXLEN)
        self._bufs = [collections.deque(maxlen=GRAPH_MAXLEN) for _ in range(n)]

        for ax, (label, unit, color) in zip(self._axes, _GRAPH_PLOTS):
            ax.set_facecolor('#141414')
            ax.tick_params(colors='#555555', labelsize=8, length=3)
            ax.set_ylabel(f'{label}\n({unit})', color='#555555', fontsize=8,
                          rotation=0, labelpad=44, va='center')
            for spine in ax.spines.values():
                spine.set_edgecolor('#2a2a2a')
            line, = ax.plot([], [], color=color, linewidth=1.0, antialiased=True)
            self._lines.append(line)

        self._axes[-1].set_xlabel('Time (s)', color='#555555', fontsize=8)
        self._fig.subplots_adjust(left=0.14, right=0.97, top=0.97, bottom=0.07, hspace=0.22)

    def push(self, t: float, values: list):
        """values = [pos_deg, vel_ts, cur_a, temp_c_or_None, voltage_v_or_None]"""
        self._buf_t.append(t)
        for buf, v in zip(self._bufs, values):
            buf.append(float('nan') if v is None else float(v))

    def redraw(self):
        if len(self._buf_t) < 2:
            return
        t0 = self._buf_t[0]
        t_arr = [x - t0 for x in self._buf_t]
        for ax, line, buf in zip(self._axes, self._lines, self._bufs):
            line.set_data(t_arr, list(buf))
            ax.relim()
            ax.autoscale_view(scalex=True, scaley=True)
        self.draw_idle()

    def clear(self):
        self._buf_t.clear()
        for buf in self._bufs:
            buf.clear()
        for line in self._lines:
            line.set_data([], [])
        self.draw_idle()


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


# ── Per-motor channel (state + publishers + send helpers) ─────────────────────

@dataclass
class MotorChannel:
    name: str
    topic_ns: str                    # e.g. '/gim8108_motor_node' or '/motor_1/gim8108_motor_node'
    process: Optional[Any] = None   # subprocess.Popen for GUI-launched nodes

    # Launch parameters (persisted to config)
    param_axis: int = 0
    param_gear_ratio: float = 8.0
    param_usb_path: str = ''
    param_serial: str = ''

    # GUI panel settings (soft limits, speed, accel, current) — saved to PANEL_CONFIG
    gui_settings: dict = field(default_factory=dict)

    # State
    pos_deg: float = 0.0
    pos_abs_deg: float = 0.0
    vel_out_ts: float = 0.0
    current_a: float = 0.0
    voltage_v: Optional[float] = None
    temperature: Optional[float] = None
    is_connected: bool = False
    serial_number: str = ''
    _cfg: dict = field(default_factory=dict)

    # Callbacks — card (always active, set by add_card, never overwritten)
    on_card_state: Any = None
    on_card_connected: Any = None

    # Callbacks — detail panel (only when motor is selected)
    on_state: Any = None
    on_connected: Any = None
    on_calib: Any = None
    on_config: Any = None
    on_serial: Any = None

    # ROS publishers (populated by MultiGuiNode.add_channel)
    pub_pos:          Any = None
    pub_vel:          Any = None
    pub_torque:       Any = None
    pub_vel_lim:      Any = None
    pub_cur_lim:      Any = None
    pub_traj_vel:     Any = None
    pub_mode:         Any = None
    pub_pos_gain:     Any = None
    pub_vel_gain:     Any = None
    pub_vel_int_gain: Any = None
    pub_traj_accel:   Any = None
    pub_traj_decel:   Any = None
    enable_cli:  Any = None
    clear_cli:   Any = None
    calib_cli:   Any = None
    save_cli:    Any = None
    estop_cli:   Any = None

    # ── Send helpers ──────────────────────────────────────────────────────────

    def _pub(self, pub, val, MsgType):
        m = MsgType(); m.data = val; pub.publish(m)

    def send_pos_deg(self, v):     self._pub(self.pub_pos,          v, Float64)
    def send_vel(self, v):         self._pub(self.pub_vel,          v, Float64)
    def send_torque(self, v):      self._pub(self.pub_torque,       v, Float64)
    def send_vel_limit(self, v):   self._pub(self.pub_vel_lim,      v, Float64)
    def send_cur_limit(self, v):   self._pub(self.pub_cur_lim,      v, Float64)
    def send_traj_vel(self, v):    self._pub(self.pub_traj_vel,     v, Float64)
    def send_mode(self, v):        self._pub(self.pub_mode,         v, Int32)
    def send_pos_gain(self, v):    self._pub(self.pub_pos_gain,     v, Float64)
    def send_vel_gain(self, v):    self._pub(self.pub_vel_gain,     v, Float64)
    def send_vel_int_gain(self, v):self._pub(self.pub_vel_int_gain, v, Float64)
    def send_traj_accel(self, v):  self._pub(self.pub_traj_accel,  v, Float64)
    def send_traj_decel(self, v):  self._pub(self.pub_traj_decel,  v, Float64)

    def _call(self, client, request, label):
        if not client.service_is_ready():
            return f'{label}: driver not ready'
        client.call_async(request)
        return None

    def call_enable(self, enable: bool):
        req = SetBool.Request(); req.data = enable
        return self._call(self.enable_cli, req, 'enable')

    def call_clear(self):   return self._call(self.clear_cli,  Trigger.Request(), 'clear_errors')
    def call_calibrate(self): return self._call(self.calib_cli, Trigger.Request(), 'calibrate')
    def call_save(self):    return self._call(self.save_cli,   Trigger.Request(), 'save_config')
    def call_estop(self):   return self._call(self.estop_cli,  Trigger.Request(), 'estop')


# ── ROS node (manages all channels) ──────────────────────────────────────────

class MultiGuiNode(Node):
    def __init__(self):
        super().__init__('gim8108_gui')
        self.channels: Dict[str, MotorChannel] = {}

    def add_channel(self, ch: MotorChannel):
        ns = ch.topic_ns
        ch.pub_pos          = self.create_publisher(Float64, f'{ns}/cmd_pos_deg',        10)
        ch.pub_vel          = self.create_publisher(Float64, f'{ns}/cmd_vel',             10)
        ch.pub_torque       = self.create_publisher(Float64, f'{ns}/cmd_torque',          10)
        ch.pub_vel_lim      = self.create_publisher(Float64, f'{ns}/vel_limit',           10)
        ch.pub_cur_lim      = self.create_publisher(Float64, f'{ns}/current_limit',       10)
        ch.pub_traj_vel     = self.create_publisher(Float64, f'{ns}/traj_vel_limit',      10)
        ch.pub_mode         = self.create_publisher(Int32,   f'{ns}/control_mode',        10)
        ch.pub_pos_gain     = self.create_publisher(Float64, f'{ns}/pos_gain',            10)
        ch.pub_vel_gain     = self.create_publisher(Float64, f'{ns}/vel_gain',            10)
        ch.pub_vel_int_gain = self.create_publisher(Float64, f'{ns}/vel_integrator_gain', 10)
        ch.pub_traj_accel   = self.create_publisher(Float64, f'{ns}/traj_accel_limit',    10)
        ch.pub_traj_decel   = self.create_publisher(Float64, f'{ns}/traj_decel_limit',    10)

        ch.enable_cli = self.create_client(SetBool, f'{ns}/enable')
        ch.clear_cli  = self.create_client(Trigger,  f'{ns}/clear_errors')
        ch.calib_cli  = self.create_client(Trigger,  f'{ns}/calibrate')
        ch.save_cli   = self.create_client(Trigger,  f'{ns}/save_config')
        ch.estop_cli  = self.create_client(Trigger,  f'{ns}/estop')

        self.create_subscription(JointState, f'{ns}/joint_state',
                                 lambda m, c=ch: self._on_state(m, c),     10)
        self.create_subscription(Bool,       f'{ns}/is_connected',
                                 lambda m, c=ch: self._on_connected(m, c), _QOS_LATCHED)
        self.create_subscription(String,     f'{ns}/calib_status',
                                 lambda m, c=ch: self._on_calib(m, c),     10)
        self.create_subscription(String,     f'{ns}/serial_number',
                                 lambda m, c=ch: self._on_serial(m, c),    _QOS_LATCHED)
        self.create_subscription(Float64,    f'{ns}/bus_voltage',
                                 lambda m, c=ch: self._on_voltage(m, c),   10)
        self.create_subscription(Float64,    f'{ns}/temperature',
                                 lambda m, c=ch: self._on_temperature(m, c), 10)

        for key, topic in (
            ('pos_gain',   f'{ns}/config/pos_gain'),
            ('vel_gain',   f'{ns}/config/vel_gain'),
            ('vel_int',    f'{ns}/config/vel_integrator_gain'),
            ('traj_vel',   f'{ns}/config/traj_vel_limit'),
            ('traj_accel', f'{ns}/config/traj_accel_limit'),
            ('traj_decel', f'{ns}/config/traj_decel_limit'),
            ('vel_limit',  f'{ns}/config/vel_limit'),
            ('cur_limit',  f'{ns}/config/current_lim'),
        ):
            self.create_subscription(Float64, topic,
                                     lambda m, c=ch, k=key: self._on_cfg(m, c, k),
                                     _QOS_LATCHED)

        self.channels[ns] = ch

    def _on_state(self, msg: JointState, ch: MotorChannel):
        if msg.position:
            ch.pos_abs_deg = math.degrees(msg.position[0])
            ch.pos_deg = ch.pos_abs_deg % 360.0
        if msg.velocity:
            ch.vel_out_ts = msg.velocity[0] / (2.0 * math.pi)
        if msg.effort:
            ch.current_a = msg.effort[0]
        if ch.on_card_state: ch.on_card_state()
        if ch.on_state: ch.on_state()

    def _on_voltage(self, msg: Float64, ch: MotorChannel):
        ch.voltage_v = msg.data
        if ch.on_card_state: ch.on_card_state()
        if ch.on_state: ch.on_state()

    def _on_temperature(self, msg: Float64, ch: MotorChannel):
        ch.temperature = msg.data
        if ch.on_state: ch.on_state()

    def _on_connected(self, msg: Bool, ch: MotorChannel):
        if ch.is_connected != msg.data:
            ch.is_connected = msg.data
            if ch.on_card_connected: ch.on_card_connected()
            if ch.on_connected: ch.on_connected()

    def _on_calib(self, msg: String, ch: MotorChannel):
        if ch.on_calib: ch.on_calib(msg.data)

    def _on_serial(self, msg: String, ch: MotorChannel):
        ch.serial_number = msg.data
        if ch.on_serial: ch.on_serial(msg.data)

    def _on_cfg(self, msg: Float64, ch: MotorChannel, key: str):
        ch._cfg[key] = msg.data
        if ch.on_config: ch.on_config(key, msg.data)


# ── Add Motor dialog ──────────────────────────────────────────────────────────

class AddMotorDialog(QDialog):
    _scan_done = pyqtSignal(list)   # list of serial number strings

    def __init__(self, parent, next_index: int):
        super().__init__(parent)
        self.setWindowTitle('Add Motor')
        self.setMinimumWidth(360)

        form = QFormLayout(self)
        form.setSpacing(10)

        self.name_edit = QLineEdit(f'Motor {next_index}')
        form.addRow('Display name:', self.name_edit)

        self.axis_combo = QComboBox()
        self.axis_combo.addItems(['Axis 0', 'Axis 1'])
        form.addRow('ODrive axis:', self.axis_combo)

        self.gear_spin = QDoubleSpinBox()
        self.gear_spin.setRange(1.0, 100.0)
        self.gear_spin.setValue(8.0)
        self.gear_spin.setSingleStep(0.5)
        self.gear_spin.setSuffix(' : 1')
        form.addRow('Gear ratio:', self.gear_spin)

        # Serial number row: text field + Scan button
        serial_row = QHBoxLayout()
        self.serial_combo = QComboBox()
        self.serial_combo.setEditable(True)
        self.serial_combo.lineEdit().setPlaceholderText('blank = auto-detect')
        self.btn_scan = QPushButton('🔍 Scan')
        self.btn_scan.setFixedWidth(80)
        self.btn_scan.clicked.connect(self._start_scan)
        serial_row.addWidget(self.serial_combo, 1)
        serial_row.addWidget(self.btn_scan)
        form.addRow('USB Serial No.:', serial_row)

        self._scan_done.connect(self._on_scan_done)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        form.addRow(btns)

    def _start_scan(self):
        self.btn_scan.setEnabled(False)
        self.btn_scan.setText('Scanning…')
        threading.Thread(target=self._do_scan, daemon=True).start()

    def _do_scan(self):
        # Output format per line: "BUS:ADDR|SERIAL"  (serial empty if unreadable)
        script = (
            'import usb.core\n'
            'devs = list(usb.core.find(idVendor=0x1209, idProduct=0x0d32, find_all=True) or [])\n'
            'for d in devs:\n'
            '  try:\n'
            '    sn = ""\n'
            '    try:\n'
            '      raw = d.serial_number\n'
            '      int(raw, 16)  # only valid hex\n'
            '      sn = raw.upper()\n'
            '    except Exception: pass\n'
            '    print(f"{d.bus:03d}:{d.address:03d}|{sn}")\n'
            '  except Exception: pass\n'
        )
        try:
            result = subprocess.run(
                ['python3', '-c', script],
                capture_output=True, text=True, timeout=10
            )
            entries = [s.strip() for s in result.stdout.splitlines() if s.strip()]
        except Exception:
            entries = []
        self._scan_done.emit(entries)

    def _on_scan_done(self, entries: list):
        self.btn_scan.setEnabled(True)
        self.btn_scan.setText('🔍 Scan')
        if not entries:
            QMessageBox.information(self, 'Not found',
                'No ODrive detected.\n'
                'Make sure it is attached:\n'
                '  usbipd attach --wsl --busid <BUSID>\n\n'
                'Run  lsusb  to confirm ID 1209:0d32 appears.')
            return
        self.serial_combo.clear()
        for entry in entries:
            parts = entry.split('|', 1)
            usb_addr = parts[0]              # '001:017'
            sn       = parts[1] if len(parts) > 1 else ''
            bus, dev = usb_addr.split(':')
            label = f'Bus {bus}  Dev {dev}'
            if sn:
                label += f'  │  SN: {sn}'
            self.serial_combo.addItem(label, {'usb_path': usb_addr, 'serial': sn})

    @property
    def motor_name(self): return self.name_edit.text().strip() or 'Motor'
    @property
    def axis(self): return self.axis_combo.currentIndex()
    @property
    def gear_ratio(self): return self.gear_spin.value()

    @property
    def usb_path(self) -> str:
        """Return 'bus:addr' e.g. '001:017' from scan selection or manual entry."""
        data = self.serial_combo.currentData()
        if data and data.get('usb_path'):
            return data['usb_path']
        text = self.serial_combo.currentText().strip()
        if re.match(r'^\d{3}:\d{3}$', text):
            return text
        return ''

    @property
    def serial_number(self) -> str:
        """Return hex serial string from scan selection or manual entry."""
        data = self.serial_combo.currentData()
        if data and data.get('serial'):
            return data['serial']
        text = self.serial_combo.currentText().strip()
        if not re.match(r'^\d{3}:\d{3}$', text):
            return text  # manual hex serial
        return ''


# ── Motor card (left panel) ───────────────────────────────────────────────────

class MotorCard(QWidget):
    clicked = pyqtSignal()
    remove_requested = pyqtSignal()
    # Thread-safe refresh: ROS2 background thread emits this,
    # Qt queues the call and runs refresh() on the main thread.
    _refresh_sig = pyqtSignal()

    def __init__(self, ch: MotorChannel):
        super().__init__()
        self.ch = ch
        self.setFixedHeight(64)
        self.setCursor(Qt.PointingHandCursor)
        self._selected = False

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(8)

        self.dot = QLabel('⬤')
        self.dot.setFont(QFont('', 14))
        self.dot.setFixedWidth(18)

        info = QVBoxLayout()
        info.setSpacing(2)
        self.lbl_name = QLabel(ch.name)
        self.lbl_name.setFont(QFont('', 10, QFont.Bold))
        self.lbl_angle = QLabel('---')
        self.lbl_angle.setFont(QFont('Monospace', 9))

        info.addWidget(self.lbl_name)
        info.addWidget(self.lbl_angle)

        btn_del = QPushButton('✕')
        btn_del.setFixedSize(22, 22)
        btn_del.setStyleSheet(
            'QPushButton{background:transparent;border:none;color:#444;font-size:11px;}'
            'QPushButton:hover{color:#999;}'
        )
        btn_del.clicked.connect(self.remove_requested)

        layout.addWidget(self.dot)
        layout.addLayout(info)
        layout.addStretch()
        layout.addWidget(btn_del)

        self._refresh_sig.connect(self.refresh)
        self._set_disconnected()

    def refresh(self):
        if self.ch.is_connected:
            self.dot.setStyleSheet('color:#888888;')
            self.lbl_angle.setText(f'{self.ch.pos_deg:6.1f}°')
            self.lbl_name.setStyleSheet('color:#dddddd;')
        else:
            self._set_disconnected()

    def _set_disconnected(self):
        self.dot.setStyleSheet('color:#333333;')
        self.lbl_angle.setText('---')
        self.lbl_name.setStyleSheet('color:#666666;')

    def set_selected(self, sel: bool):
        self._selected = sel
        self.setStyleSheet(
            'MotorCard { background:#2a2a2a; border-left:2px solid #888; border-radius:3px; }' if sel else
            'MotorCard { background:transparent; border-left:2px solid transparent; border-radius:3px; }'
        )

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit()


# ── Left panel: motor list ────────────────────────────────────────────────────

class MotorListPanel(QWidget):
    motor_selected = pyqtSignal(str)   # emits topic_ns

    def __init__(self, node: MultiGuiNode):
        super().__init__()
        self.node = node
        self.cards: Dict[str, MotorCard] = {}
        self._selected_ns: Optional[str] = None
        self._next_motor_idx = 1
        MOTORS_CONFIG.parent.mkdir(parents=True, exist_ok=True)
        self.setFixedWidth(220)
        self._build()

    def _build(self):
        v = QVBoxLayout(self)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(4)

        header = QLabel('MOTORS')
        header.setStyleSheet('color:#555555; font-size:10px; font-weight:bold;')
        header.setContentsMargins(6, 4, 0, 4)
        v.addWidget(header)

        sep = QFrame(); sep.setFrameShape(QFrame.HLine)
        v.addWidget(sep)

        # Scrollable card area
        self._card_widget = QWidget()
        self._card_layout = QVBoxLayout(self._card_widget)
        self._card_layout.setContentsMargins(0, 0, 0, 0)
        self._card_layout.setSpacing(2)
        self._card_layout.addStretch()

        scroll = QScrollArea()
        scroll.setWidget(self._card_widget)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        v.addWidget(scroll, 1)

        sep2 = QFrame(); sep2.setFrameShape(QFrame.HLine)
        v.addWidget(sep2)

        btn_add = QPushButton('＋  Add Motor')
        btn_add.clicked.connect(self._on_add)
        v.addWidget(btn_add)

    def add_card(self, ch: MotorChannel):
        card = MotorCard(ch)
        card.clicked.connect(lambda ns=ch.topic_ns: self._select(ns))
        card.remove_requested.connect(lambda ns=ch.topic_ns: self._remove_motor(ns))
        # Card callbacks — emit via signal so refresh() runs on the main thread,
        # not the ROS2 background thread (which would corrupt Qt's heap).
        ch.on_card_state     = card._refresh_sig.emit
        ch.on_card_connected = card._refresh_sig.emit
        self.cards[ch.topic_ns] = card
        # Insert before the stretch
        idx = self._card_layout.count() - 1
        self._card_layout.insertWidget(idx, card)
        if not self._selected_ns:
            self._select(ch.topic_ns)

    def _select(self, ns: str):
        if self._selected_ns and self._selected_ns in self.cards:
            self.cards[self._selected_ns].set_selected(False)
        self._selected_ns = ns
        if ns in self.cards:
            self.cards[ns].set_selected(True)
        self.motor_selected.emit(ns)

    def refresh_card(self, ns: str):
        if ns in self.cards:
            self.cards[ns].refresh()

    def _on_add(self):
        dlg = AddMotorDialog(self, self._next_motor_idx)
        if dlg.exec_() != QDialog.Accepted:
            return

        idx  = self._next_motor_idx
        self._next_motor_idx += 1
        ns   = f'/motor_{idx}/gim8108_motor_node'
        name = dlg.motor_name
        axis = dlg.axis
        gr   = dlg.gear_ratio
        usb  = dlg.usb_path
        sn   = dlg.serial_number

        proc = self._launch_node(ns, axis, gr, usb, sn)
        if proc is None:
            return

        ch = MotorChannel(name=name, topic_ns=ns, process=proc,
                          param_axis=axis, param_gear_ratio=gr,
                          param_usb_path=usb, param_serial=sn)
        self.node.add_channel(ch)
        self.add_card(ch)
        self._save_config()

    def _launch_node(self, ns: str, axis: int, gr: float, usb_path: str, serial: str):
        cmd = [
            'ros2', 'run', 'gim8108_driver', 'gim8108_motor_node_usb',
            '--ros-args',
            '--remap', f'__ns:={ns.replace("/gim8108_motor_node", "")}',
            '-p', f'axis:={axis}',
            '-p', f'gear_ratio:={gr}',
            '-p', 'home_on_connect:=true',
        ]
        if usb_path:
            cmd += ['-p', f'usb_path:={usb_path}']
        elif serial:
            cmd += ['-p', f'serial_number:={serial}']
        try:
            return subprocess.Popen(cmd)
        except FileNotFoundError:
            QMessageBox.critical(self, 'Error', 'ros2 not found. Make sure ROS2 is sourced.')
            return None

    def _remove_motor(self, ns: str):
        ch = self.node.channels.get(ns)
        if not ch:
            return
        reply = QMessageBox.question(
            self, 'Remove Motor', f'Remove "{ch.name}" ?',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        # Clear callbacks before teardown to avoid use-after-free
        ch.on_card_state     = None
        ch.on_card_connected = None
        ch.on_state          = None
        ch.on_connected      = None
        ch.on_calib          = None
        ch.on_config         = None
        ch.on_serial         = None

        if ch.process:
            try:
                ch.process.terminate()
            except Exception:
                pass

        self.node.channels.pop(ns, None)

        card = self.cards.pop(ns, None)
        if card:
            self._card_layout.removeWidget(card)
            card.deleteLater()

        if self._selected_ns == ns:
            self._selected_ns = None
            if self.cards:
                self._select(next(iter(self.cards)))
            else:
                self.motor_selected.emit('')

        self._save_config()

    def save_panel_settings(self):
        """Persist GUI panel settings for all motors to PANEL_CONFIG."""
        settings = {ns: ch.gui_settings
                    for ns, ch in self.node.channels.items()
                    if ch.gui_settings}
        try:
            PANEL_CONFIG.write_text(json.dumps(settings, indent=2))
        except Exception:
            pass

    def _load_panel_settings(self) -> dict:
        if not PANEL_CONFIG.exists():
            return {}
        try:
            return json.loads(PANEL_CONFIG.read_text())
        except Exception:
            return {}

    def _save_config(self):
        """Save user-added motors (not Motor 0) to disk."""
        saved = []
        for ns, ch in self.node.channels.items():
            if ns == DEFAULT_NS:
                continue  # default motor is always added at startup
            saved.append({
                'name':       ch.name,
                'topic_ns':   ns,
                'axis':       ch.param_axis,
                'gear_ratio': ch.param_gear_ratio,
                'usb_path':   ch.param_usb_path,
                'serial':     ch.param_serial,
            })
        try:
            MOTORS_CONFIG.write_text(json.dumps(saved, indent=2))
        except Exception:
            pass

    def _load_and_relaunch(self):
        """Reload and relaunch motors saved from the previous session."""
        if not MOTORS_CONFIG.exists():
            return
        # Kill any USB motor node processes left over from a previous session
        # so they release the ODrive USB device before we relaunch.
        subprocess.run(['pkill', '-f', 'gim8108_motor_node_usb'], capture_output=True)
        time.sleep(1.0)
        try:
            saved = json.loads(MOTORS_CONFIG.read_text())
        except Exception:
            return
        panel_settings = self._load_panel_settings()
        for cfg in saved:
            ns = cfg.get('topic_ns', '')
            if not ns or ns == DEFAULT_NS or ns in self.node.channels:
                continue
            # Update index counter to avoid collision with new additions
            m = re.search(r'/motor_(\d+)/', ns)
            if m:
                self._next_motor_idx = max(self._next_motor_idx, int(m.group(1)) + 1)
            axis = cfg.get('axis', 0)
            gr   = cfg.get('gear_ratio', 8.0)
            usb  = cfg.get('usb_path', '')
            sn   = cfg.get('serial', '')
            proc = self._launch_node(ns, axis, gr, usb, sn)
            if proc is None:
                continue
            ch = MotorChannel(name=cfg.get('name', 'Motor'), topic_ns=ns, process=proc,
                              param_axis=axis, param_gear_ratio=gr,
                              param_usb_path=usb, param_serial=sn,
                              gui_settings=panel_settings.get(ns, {}))
            self.node.add_channel(ch)
            self.add_card(ch)


# ── Motor Control detail panel ────────────────────────────────────────────────

class MotorDetailPanel(QWidget):
    _sig_state     = pyqtSignal()
    _sig_connected = pyqtSignal()
    _sig_calib     = pyqtSignal(str)
    _sig_config    = pyqtSignal(str, float)
    _sig_serial    = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.ch: Optional[MotorChannel] = None

        self._current_mode: int = CTRL_POSITION

        # Motion state (shared across channel switches)
        self._recorded: List[float] = []
        self._record_hz: float = 10.0
        self._record_timer = QTimer()
        self._record_timer.timeout.connect(self._record_frame)
        self._motion: Optional[Motion] = None
        self._play_idx: int = 0
        self._play_timer = QTimer()
        self._play_timer.timeout.connect(self._play_step)
        self._play_done_callback = None  # called by _stop_play when playback finishes

        # Sequencer state
        self._seq_items: List[dict] = []    # [{'name': str, 'delay': float}, ...]
        self._seq_idx: int = 0
        self._seq_running: bool = False
        self._seq_start_t: float = 0.0
        self._seq_delay_timer = QTimer()
        self._seq_delay_timer.setSingleShot(True)
        self._seq_delay_timer.timeout.connect(self._seq_next)
        self._seq_update_timer = QTimer()
        self._seq_update_timer.timeout.connect(self._seq_update_progress)

        # Graph state
        self._graph_canvas: Optional[StatusGraphCanvas] = None
        self._graph_timer = QTimer()
        self._graph_timer.timeout.connect(self._graph_update)
        self._graph_t0: float = time.monotonic()

        MOTION_DIR.mkdir(exist_ok=True)
        SEQ_DIR.mkdir(exist_ok=True)

        self._sig_state.connect(self._refresh_status)
        self._sig_connected.connect(self._refresh_connection)
        self._sig_calib.connect(self._on_calib_msg)
        self._sig_config.connect(self._on_config_update)
        self._sig_serial.connect(self._on_serial_update)

        self._build()
        self._refresh_motion_list()
        self._seq_refresh_saved()

    # ── Channel switching ─────────────────────────────────────────────────────

    def set_channel(self, ch: MotorChannel):
        if self.ch:
            self._save_settings_to_channel(self.ch)
            self.ch.on_state     = None
            self.ch.on_connected = None
            self.ch.on_calib     = None
            self.ch.on_config    = None
            self.ch.on_serial    = None
        self.ch = ch
        if ch:
            ch.on_state     = self._sig_state.emit
            ch.on_connected = self._sig_connected.emit
            ch.on_calib     = self._sig_calib.emit
            ch.on_config    = self._sig_config.emit
            ch.on_serial    = self._sig_serial.emit
            self._restore_settings_from_channel(ch)
            self.lbl_motor_header.setText(f'  {ch.name}   {ch.topic_ns}')
            self.lbl_motor_header.setStyleSheet(
                'background:#202020; color:#cccccc; padding:7px 14px; '
                'font-size:13px; font-weight:bold; '
                'border-bottom-width:1px; border-bottom-style:solid; border-bottom-color:#333333;'
            )
        else:
            self.lbl_motor_header.setText('  No motor selected')
            self.lbl_motor_header.setStyleSheet(
                'background:#202020; color:#555555; padding:7px 14px; '
                'font-size:13px; '
                'border-bottom-width:1px; border-bottom-style:solid; border-bottom-color:#333333;'
            )
        self._refresh_connection()
        self._refresh_status()

    # ── UI build ──────────────────────────────────────────────────────────────

    def _build(self):
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        self.lbl_motor_header = QLabel('  No motor selected')
        self.lbl_motor_header.setStyleSheet(
            'background:#202020; color:#555555; padding:7px 14px; '
            'font-size:13px; font-weight:bold; '
            'border-bottom-width:1px; border-bottom-style:solid; border-bottom-color:#333333;'
        )
        v.addWidget(self.lbl_motor_header)

        v.addWidget(self._make_status_bar())
        tabs = QTabWidget()
        tabs.addTab(self._make_control_tab(),    'Motor Control')
        tabs.addTab(self._make_motion_tab(),     'Motion')
        tabs.addTab(self._make_seq_tab(),        'Sequencer')
        tabs.addTab(self._make_status_graph_tab(), 'Status')
        v.addWidget(tabs)

    # ── Status bar ────────────────────────────────────────────────────────────

    def _make_status_bar(self):
        grp = QGroupBox('Status')
        h = QHBoxLayout(grp)
        h.setSpacing(20)

        self.lbl_conn = QLabel('⬤  DISCONNECTED')
        self.lbl_conn.setFont(QFont('', 11, QFont.Bold))
        self.lbl_conn.setStyleSheet('color:#444444;')
        self.lbl_conn.setMinimumWidth(160)

        sep = QFrame(); sep.setFrameShape(QFrame.VLine)

        self.lbl_pos    = QLabel('Angle    ---')
        self.lbl_vel    = QLabel('Speed    ---')
        self.lbl_cur    = QLabel('Current  ---')
        self.lbl_vbus   = QLabel('Voltage  ---')
        self.lbl_temp   = QLabel('Temp     ---')
        self.lbl_serial = QLabel('')
        mono = QFont('Monospace', 10)
        for lbl in (self.lbl_pos, self.lbl_vel, self.lbl_cur, self.lbl_vbus, self.lbl_temp):
            lbl.setFont(mono)
            lbl.setMinimumWidth(140)
            lbl.setStyleSheet('color:#555555;')
        self.lbl_serial.setFont(QFont('Monospace', 9))
        self.lbl_serial.setStyleSheet('color:#3a3a3a;')

        h.addWidget(self.lbl_conn)
        h.addWidget(sep)
        for lbl in (self.lbl_pos, self.lbl_vel, self.lbl_cur, self.lbl_vbus, self.lbl_temp):
            h.addWidget(lbl)
        h.addStretch()
        h.addWidget(self.lbl_serial)
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
        v.setSpacing(6)

        h1 = QHBoxLayout()
        self.btn_enable = QPushButton('Enable Motor')
        self.btn_enable.setCheckable(True)
        self.btn_enable.setStyleSheet(
            'QPushButton:checked{background:#606060;color:#ffffff;font-weight:bold;border-color:#999;}'
        )
        self.btn_enable.toggled.connect(self._on_enable)

        self.btn_free_ctrl = QPushButton('Free  (Torque OFF)')
        self.btn_free_ctrl.setCheckable(True)
        self.btn_free_ctrl.setStyleSheet(
            'QPushButton:checked{background:#454545;color:#dddddd;font-weight:bold;border:1px solid #888;}'
        )
        self.btn_free_ctrl.toggled.connect(self._on_free_ctrl_toggled)

        btn_home = QPushButton('⌂  Origin (0°)')
        btn_home.setToolTip('Enable motor and move to 0° via shortest path')
        btn_home.setStyleSheet('background:#303030;color:#cccccc;font-weight:bold;border:1px solid #666;')
        btn_home.clicked.connect(self._on_home)

        btn_clear = QPushButton('Clear Errors')
        btn_clear.clicked.connect(self._on_clear)

        btn_estop = QPushButton('⚠  E-STOP')
        btn_estop.setStyleSheet('background:#cccccc;color:#000000;font-weight:bold;font-size:12px;border:none;')
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
        btn_save.setStyleSheet('background:#2e2e2e;color:#aaaaaa;border:1px solid #666;')
        btn_save.clicked.connect(self._on_save)
        h2.addWidget(btn_calib)
        h2.addWidget(btn_save)
        h2.addStretch()
        v.addLayout(h2)
        return grp

    def _make_mode_group(self):
        grp = QGroupBox('Control Mode')
        h = QHBoxLayout(grp)
        _mode_style = (
            'QPushButton{min-height:30px;font-size:12px;}'
            'QPushButton:checked{background:#505050;color:#ffffff;font-weight:bold;border:1px solid #aaa;}'
        )
        self.btn_mode_pos = QPushButton('⌖  Position')
        self.btn_mode_vel = QPushButton('⟳  Velocity')
        self.btn_mode_trq = QPushButton('◎  Torque')
        for btn in (self.btn_mode_pos, self.btn_mode_vel, self.btn_mode_trq):
            btn.setCheckable(True)
            btn.setStyleSheet(_mode_style)
            h.addWidget(btn)
        self.btn_mode_pos.setChecked(True)
        self.btn_mode_pos.clicked.connect(lambda: self._switch_mode(CTRL_POSITION))
        self.btn_mode_vel.clicked.connect(lambda: self._switch_mode(CTRL_VELOCITY))
        self.btn_mode_trq.clicked.connect(lambda: self._switch_mode(CTRL_TORQUE))
        return grp

    def _make_pos_group(self):
        grp = QGroupBox('Target Position  [degrees]')
        v = QVBoxLayout(grp)
        v.setSpacing(6)

        # Slider (0–360°) — quick single-turn control
        hs = QHBoxLayout()
        self.slider_pos = QSlider(Qt.Horizontal)
        self.slider_pos.setRange(0, 3600)
        self.slider_pos.setTickPosition(QSlider.TicksBelow)
        self.slider_pos.setTickInterval(900)
        self.slider_pos.valueChanged.connect(self._on_pos_slider)
        hs.addWidget(self.slider_pos)
        v.addLayout(hs)
        tick_row = QHBoxLayout()
        for t in ('0°', '90°', '180°', '270°', '360°'):
            tick_row.addWidget(QLabel(t))
            if t != '360°': tick_row.addStretch()
        v.addLayout(tick_row)

        # Absolute position spinbox (multi-turn)
        h1 = QHBoxLayout()
        h1.addWidget(QLabel('Absolute:'))
        self.spin_pos = QDoubleSpinBox()
        self.spin_pos.setRange(-36000.0, 36000.0)
        self.spin_pos.setSingleStep(1.0)
        self.spin_pos.setDecimals(1)
        self.spin_pos.setSuffix(' °')
        self.spin_pos.setFixedWidth(130)
        self.spin_pos.valueChanged.connect(self._on_pos_spin)
        btn_sync = QPushButton('⟳ Sync')
        btn_sync.setFixedWidth(70)
        btn_sync.setToolTip('Set target to current motor position')
        btn_sync.clicked.connect(self._on_pos_sync)
        h1.addWidget(self.spin_pos)
        h1.addWidget(btn_sync)
        h1.addStretch()
        v.addLayout(h1)

        # Relative move buttons
        h2 = QHBoxLayout()
        h2.addWidget(QLabel('Move:'))
        self._rel_move_btns = []
        for delta in (-90, -10, -1, +1, +10, +90):
            btn = QPushButton(f'{delta:+d}°')
            btn.setFixedWidth(54)
            btn.clicked.connect(lambda _, d=delta: self._on_rel_move(d))
            h2.addWidget(btn)
            self._rel_move_btns.append(btn)
        h2.addStretch()
        v.addLayout(h2)

        # Software position limits
        h3 = QHBoxLayout()
        self.chk_limit = QCheckBox('Soft limit')
        self.spin_limit_min = QDoubleSpinBox()
        self.spin_limit_min.setRange(-36000.0, 36000.0)
        self.spin_limit_min.setValue(-180.0)
        self.spin_limit_min.setSuffix(' °')
        self.spin_limit_min.setFixedWidth(110)
        self.spin_limit_min.setEnabled(False)
        self.spin_limit_max = QDoubleSpinBox()
        self.spin_limit_max.setRange(-36000.0, 36000.0)
        self.spin_limit_max.setValue(180.0)
        self.spin_limit_max.setSuffix(' °')
        self.spin_limit_max.setFixedWidth(110)
        self.spin_limit_max.setEnabled(False)
        self.chk_limit.toggled.connect(self._on_limit_toggled)
        h3.addWidget(self.chk_limit)
        h3.addWidget(QLabel('Min:'))
        h3.addWidget(self.spin_limit_min)
        h3.addWidget(QLabel('Max:'))
        h3.addWidget(self.spin_limit_max)
        h3.addStretch()
        v.addLayout(h3)
        return grp

    def _make_speed_group(self):
        grp = QGroupBox('Trajectory Speed  [motor turns/s]')
        v = QVBoxLayout(grp)
        h1 = QHBoxLayout()
        h1.addWidget(QLabel('Vel Limit:'))
        self.slider_spd = QSlider(Qt.Horizontal)
        self.slider_spd.setRange(1, 1000)
        self.slider_spd.setValue(100)
        self.slider_spd.valueChanged.connect(self._on_spd_slider)
        self.lbl_spd = QLabel('10.0 t/s')
        self.lbl_spd.setFixedWidth(75)
        h1.addWidget(self.slider_spd)
        h1.addWidget(self.lbl_spd)
        v.addLayout(h1)
        row = QHBoxLayout()
        for t in ('0.1', '25', '50', '75', '100 t/s'):
            row.addWidget(QLabel(t))
            if 't/s' not in t: row.addStretch()
        v.addLayout(row)
        h2 = QHBoxLayout()
        h2.addWidget(QLabel('Accel:'))
        self.spin_accel = QDoubleSpinBox()
        self.spin_accel.setRange(0.5, 200.0); self.spin_accel.setValue(5.0)
        self.spin_accel.setSingleStep(1.0);    self.spin_accel.setDecimals(1)
        self.spin_accel.setSuffix(' t/s²');    self.spin_accel.setFixedWidth(110)
        self.spin_accel.valueChanged.connect(lambda v: self.ch and self.ch.send_traj_accel(v))
        h2.addWidget(self.spin_accel)
        h2.addSpacing(16)
        h2.addWidget(QLabel('Decel:'))
        self.spin_decel = QDoubleSpinBox()
        self.spin_decel.setRange(0.5, 200.0); self.spin_decel.setValue(5.0)
        self.spin_decel.setSingleStep(1.0);    self.spin_decel.setDecimals(1)
        self.spin_decel.setSuffix(' t/s²');    self.spin_decel.setFixedWidth(110)
        self.spin_decel.valueChanged.connect(lambda v: self.ch and self.ch.send_traj_decel(v))
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
        self.lbl_cur_set.setFixedWidth(75)
        h.addWidget(self.slider_cur)
        h.addWidget(self.lbl_cur_set)
        v.addLayout(h)
        row = QHBoxLayout()
        for t in ('0.1 A', '6 A', '13 A', '19 A', '25 A'):
            row.addWidget(QLabel(t))
            if t != '25 A': row.addStretch()
        v.addLayout(row)
        return grp

    def _make_pid_group(self):
        grp = QGroupBox('PID Gains')
        grid = QGridLayout(grp)

        def spin(lo, hi, val, step, dec, sfx):
            w = QDoubleSpinBox()
            w.setRange(lo, hi); w.setValue(val); w.setSingleStep(step)
            w.setDecimals(dec); w.setSuffix(sfx); w.setFixedWidth(125)
            return w

        grid.addWidget(QLabel('Pos Gain:'), 0, 0)
        self.spin_pos_gain = spin(0, 200, 20.0, 1.0, 1, ' (t/s)/t')
        self.spin_pos_gain.valueChanged.connect(lambda v: self.ch and self.ch.send_pos_gain(v))
        grid.addWidget(self.spin_pos_gain, 0, 1)

        grid.addWidget(QLabel('Vel Gain:'), 0, 2)
        self.spin_vel_gain = spin(0, 5.0, 0.16, 0.01, 3, ' A/(t/s)')
        self.spin_vel_gain.valueChanged.connect(lambda v: self.ch and self.ch.send_vel_gain(v))
        grid.addWidget(self.spin_vel_gain, 0, 3)

        grid.addWidget(QLabel('Vel Int:'), 0, 4)
        self.spin_vel_int = spin(0, 10.0, 0.32, 0.01, 3, ' A/(t/s²)')
        self.spin_vel_int.valueChanged.connect(lambda v: self.ch and self.ch.send_vel_int_gain(v))
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
        self.calib_log.setMaximumHeight(80)
        self.calib_log.setFont(QFont('Monospace', 9))
        self.calib_log.setPlaceholderText('Calibration messages appear here...')
        v.addWidget(self.calib_log)
        return grp

    # ── Motion tab ─────────────────────────────────────────────────────────────

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
        self.btn_free_rec = QPushButton('Free  (Torque OFF)')
        self.btn_free_rec.setCheckable(True)
        self.btn_free_rec.setStyleSheet(
            'QPushButton:checked{background:#454545;color:#dddddd;font-weight:bold;border:1px solid #888;}'
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
        self.btn_rec = QPushButton('⏺  Record')
        self.btn_rec.setStyleSheet('background:#3a3a3a;color:#dddddd;font-weight:bold;border:1px solid #777;')
        self.btn_rec.clicked.connect(self._start_record)
        self.btn_stop_rec = QPushButton('⏹  Stop')
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
        self.lst.setMaximumHeight(110)
        self.lst.itemSelectionChanged.connect(self._on_motion_selected)
        v.addWidget(self.lst)

        hListBtn = QHBoxLayout()
        for label, slot in (('Refresh', self._refresh_motion_list),
                             ('Load file…', self._load_file),
                             ('Delete', self._delete_motion)):
            b = QPushButton(label); b.clicked.connect(slot); hListBtn.addWidget(b)
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
        for label, val in (('0.25x', 0.25), ('0.5x', 0.5), ('1x', 1.0), ('2x', 2.0), ('4x', 4.0)):
            self.cmb_speed.addItem(label, val)
        self.cmb_speed.setCurrentIndex(2)
        hSpd.addWidget(self.cmb_speed)
        self.chk_loop = QCheckBox('Loop')
        hSpd.addWidget(self.chk_loop)
        hSpd.addStretch()
        v.addLayout(hSpd)

        hPlay = QHBoxLayout()
        self.btn_play = QPushButton('▶  Play')
        self.btn_play.setStyleSheet('background:#3a3a3a;color:#dddddd;font-weight:bold;border:1px solid #777;')
        self.btn_play.setEnabled(False)
        self.btn_play.clicked.connect(self._start_play)
        self.btn_stop_play = QPushButton('⏹  Stop')
        self.btn_stop_play.setEnabled(False)
        self.btn_stop_play.clicked.connect(self._stop_play)
        hPlay.addWidget(self.btn_play)
        hPlay.addWidget(self.btn_stop_play)
        v.addLayout(hPlay)
        v.addStretch()
        return grp

    # ── Motor Control handlers ─────────────────────────────────────────────────

    def _on_enable(self, checked: bool):
        if not self.ch: return
        err = self.ch.call_enable(checked)
        if err:
            QMessageBox.warning(self, 'Warning', err)
            self.btn_enable.blockSignals(True)
            self.btn_enable.setChecked(not checked)
            self.btn_enable.blockSignals(False)
            return
        self.btn_enable.setText('Disable Motor' if checked else 'Enable Motor')

    def _on_clear(self):
        if self.ch:
            err = self.ch.call_clear()
            if err: QMessageBox.warning(self, 'Warning', err)

    def _on_calibrate(self):
        if not self.ch: return
        err = self.ch.call_calibrate()
        if err:
            QMessageBox.warning(self, 'Warning', err)
            return
        self.calib_log.clear()
        self.calib_log.append('Calibration started...')

    def _on_save(self):
        if not self.ch: return
        reply = QMessageBox.question(
            self, 'Save Configuration',
            'ODrive will REBOOT after saving.\nUSB will disconnect briefly (~5s).\n\nContinue?',
            QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            err = self.ch.call_save()
            if err: QMessageBox.warning(self, 'Warning', err)
            else:   self.calib_log.append('Saving config — ODrive rebooting...')

    def _on_estop(self):
        if self.ch: self.ch.call_estop()
        self.btn_enable.blockSignals(True)
        self.btn_enable.setChecked(False)
        self.btn_enable.setText('Enable Motor')
        self.btn_enable.blockSignals(False)

    def _switch_mode(self, mode: int):
        prev_mode = self._current_mode
        self._current_mode = mode
        if self.ch:
            self.ch.send_mode(mode)
            # Capture current position to stop rotation when switching vel → pos
            if mode == CTRL_POSITION and prev_mode == CTRL_VELOCITY:
                QTimer.singleShot(150, lambda: self.ch and self.ch.send_pos_deg(
                    self._apply_limit(self.ch.pos_abs_deg)))

        for btn, m in ((self.btn_mode_pos, CTRL_POSITION),
                       (self.btn_mode_vel, CTRL_VELOCITY),
                       (self.btn_mode_trq, CTRL_TORQUE)):
            btn.blockSignals(True)
            btn.setChecked(mode == m)
            btn.blockSignals(False)

        self.slider_pos.setEnabled(mode == CTRL_POSITION)
        self.spin_pos.setEnabled(mode == CTRL_POSITION)
        for btn in self._rel_move_btns:
            btn.setEnabled(mode == CTRL_POSITION)
        self.slider_spd.setEnabled(mode != CTRL_TORQUE)

    def _on_pos_slider(self, val: int):
        deg = val / 10.0
        self.spin_pos.blockSignals(True)
        self.spin_pos.setValue(deg)
        self.spin_pos.blockSignals(False)
        if self.ch and self._current_mode == CTRL_POSITION:
            self.ch.send_pos_deg(self._apply_limit(deg))

    def _on_pos_spin(self, val: float):
        # Keep slider in sync when value is in its 0–360° range
        if 0.0 <= val <= 360.0:
            self.slider_pos.blockSignals(True)
            self.slider_pos.setValue(int(val * 10))
            self.slider_pos.blockSignals(False)
        if self.ch and self._current_mode == CTRL_POSITION:
            self.ch.send_pos_deg(self._apply_limit(val))

    def _on_pos_sync(self):
        """Set spinbox to current motor position without sending a command."""
        if self.ch:
            self.spin_pos.blockSignals(True)
            self.spin_pos.setValue(self.ch.pos_abs_deg)
            self.spin_pos.blockSignals(False)

    def _on_rel_move(self, delta: float):
        if not self.ch or self._current_mode != CTRL_POSITION:
            return
        target = self._apply_limit(self.ch.pos_abs_deg + delta)
        self.spin_pos.blockSignals(True)
        self.spin_pos.setValue(target)
        self.spin_pos.blockSignals(False)
        self.ch.send_pos_deg(target)

    def _on_limit_toggled(self, checked: bool):
        self.spin_limit_min.setEnabled(checked)
        self.spin_limit_max.setEnabled(checked)

    def _apply_limit(self, deg: float) -> float:
        if self.chk_limit.isChecked():
            return max(self.spin_limit_min.value(), min(self.spin_limit_max.value(), deg))
        return deg

    def _save_settings_to_channel(self, ch: MotorChannel):
        ch.gui_settings = {
            'pos_limit_enabled': self.chk_limit.isChecked(),
            'pos_limit_min':     self.spin_limit_min.value(),
            'pos_limit_max':     self.spin_limit_max.value(),
            'traj_vel':          self.slider_spd.value() / 10.0,
            'traj_accel':        self.spin_accel.value(),
            'traj_decel':        self.spin_decel.value(),
            'cur_limit':         self.slider_cur.value() / 10.0,
            'pos_gain':          self.spin_pos_gain.value(),
            'vel_gain':          self.spin_vel_gain.value(),
            'vel_int':           self.spin_vel_int.value(),
        }

    def _restore_settings_from_channel(self, ch: MotorChannel):
        s = ch.gui_settings
        if not s:
            return

        def _set_spin(w, v):
            w.blockSignals(True); w.setValue(v); w.blockSignals(False)

        self.chk_limit.blockSignals(True)
        self.chk_limit.setChecked(s.get('pos_limit_enabled', False))
        self.chk_limit.blockSignals(False)
        self._on_limit_toggled(self.chk_limit.isChecked())

        _set_spin(self.spin_limit_min, s.get('pos_limit_min', -180.0))
        _set_spin(self.spin_limit_max, s.get('pos_limit_max',  180.0))

        if 'traj_vel' in s:
            v = s['traj_vel']
            self.slider_spd.blockSignals(True)
            self.slider_spd.setValue(int(round(v * 10)))
            self.slider_spd.blockSignals(False)
            self.lbl_spd.setText(f'{v:.1f} t/s')
        if 'traj_accel' in s:
            _set_spin(self.spin_accel, s['traj_accel'])
        if 'traj_decel' in s:
            _set_spin(self.spin_decel, s['traj_decel'])
        if 'cur_limit' in s:
            v = s['cur_limit']
            self.slider_cur.blockSignals(True)
            self.slider_cur.setValue(int(round(v * 10)))
            self.slider_cur.blockSignals(False)
            self.lbl_cur_set.setText(f'{v:.1f} A')
        if 'pos_gain' in s: _set_spin(self.spin_pos_gain, s['pos_gain'])
        if 'vel_gain' in s: _set_spin(self.spin_vel_gain, s['vel_gain'])
        if 'vel_int'  in s: _set_spin(self.spin_vel_int,  s['vel_int'])

    def _on_spd_slider(self, val: int):
        turns_s = val / 10.0
        self.lbl_spd.setText(f'{turns_s:.1f} t/s')
        if not self.ch: return
        if self._current_mode == CTRL_POSITION:   self.ch.send_traj_vel(turns_s)
        elif self._current_mode == CTRL_VELOCITY: self.ch.send_vel(turns_s)

    def _on_cur_slider(self, val: int):
        a = val / 10.0
        self.lbl_cur_set.setText(f'{a:.1f} A')
        if self.ch: self.ch.send_cur_limit(a)

    def _apply_pid(self):
        if not self.ch: return
        self.ch.send_pos_gain(self.spin_pos_gain.value())
        self.ch.send_vel_gain(self.spin_vel_gain.value())
        self.ch.send_vel_int_gain(self.spin_vel_int.value())

    def _set_free(self, checked: bool):
        if self.ch: self.ch.call_enable(not checked)
        label = 'Free  (Torque OFF)  ← 手で動かせます' if checked else 'Free  (Torque OFF)'
        for btn in (self.btn_free_ctrl, self.btn_free_rec):
            btn.blockSignals(True)
            btn.setChecked(checked)
            btn.setText(label)
            btn.blockSignals(False)

    def _on_free_ctrl_toggled(self, checked: bool): self._set_free(checked)
    def _on_free_rec_toggled(self, checked: bool):  self._set_free(checked)

    def _on_home(self):
        if not self.ch: return
        self._set_free(False)
        self.btn_enable.blockSignals(True)
        self.btn_enable.setChecked(True)
        self.btn_enable.setText('Disable Motor')
        self.btn_enable.blockSignals(False)
        self.ch.call_enable(True)
        frac = self.ch.pos_abs_deg % 360.0
        raw = self.ch.pos_abs_deg - frac if frac <= 180.0 else self.ch.pos_abs_deg + (360.0 - frac)
        target = self._apply_limit(raw)
        QTimer.singleShot(300, lambda: self.ch and self.ch.send_pos_deg(target))

    # ── Motion handlers ────────────────────────────────────────────────────────

    def _start_record(self):
        self._recorded = []
        self._record_hz = self.cmb_hz.currentData()
        self._record_timer.start(int(1000 / self._record_hz))
        self.btn_rec.setEnabled(False)
        self.btn_stop_rec.setEnabled(True)
        self.lbl_frames.setText('Recording…  0 frames')

    def _record_frame(self):
        if self.ch:
            self._recorded.append(self.ch.pos_abs_deg)
        n = len(self._recorded)
        self.lbl_frames.setText(f'Recording…  {n} frames  ({n / self._record_hz:.1f} s)')

    def _stop_record(self):
        self._record_timer.stop()
        self.btn_rec.setEnabled(True)
        self.btn_stop_rec.setEnabled(False)
        self._set_free(False)
        if self.ch: self.ch.call_enable(True)
        n = len(self._recorded)
        self.lbl_frames.setText(f'Done: {n} frames  ({n / self._record_hz:.1f} s)  — press Save to keep')

    def _save_motion(self):
        if not self._recorded:
            QMessageBox.warning(self, 'No Data', 'Record a motion first.')
            return
        raw  = self.edit_name.text().strip() or 'motion'
        name = ''.join(c if c.isalnum() or c in '-_' else '_' for c in raw)
        path = MOTION_DIR / f'{name}.json'
        if path.exists():
            if QMessageBox.question(self, 'Overwrite?', f'"{name}.json" already exists. Overwrite?',
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
        path, _ = QFileDialog.getOpenFileName(self, 'Open motion file', str(MOTION_DIR), 'JSON (*.json)')
        if path: self._load_motion(Path(path))

    def _on_motion_selected(self):
        items = self.lst.selectedItems()
        if items: self._load_motion(MOTION_DIR / f'{items[0].text()}.json')

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
        if not items: return
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
        if not self._motion or not self.ch: return
        if not self.ch.is_connected:
            QMessageBox.warning(self, 'Not Connected', 'Motor driver is not connected.')
            return
        self.ch.call_enable(True)
        self._play_idx = 0
        speed    = self.cmb_speed.currentData()
        interval = max(1, int(1000 / (self._motion.hz * speed)))
        self._play_timer.start(interval)
        self.btn_play.setEnabled(False)
        self.btn_stop_play.setEnabled(True)

    def _play_step(self):
        if not self._motion or not self.ch:
            self._stop_play(); return
        if self._play_idx >= len(self._motion.frames):
            if self.chk_loop.isChecked(): self._play_idx = 0
            else: self._stop_play(); return
        self.ch.send_pos_deg(self._motion.frames[self._play_idx])
        self.progress.setValue(self._play_idx + 1)
        self._play_idx += 1

    def _stop_play(self):
        self._play_timer.stop()
        self.btn_play.setEnabled(self._motion is not None)
        self.btn_stop_play.setEnabled(False)
        cb = self._play_done_callback
        self._play_done_callback = None
        if cb:
            cb()

    # ── Sequencer tab ──────────────────────────────────────────────────────────

    def _make_seq_tab(self):
        w = QWidget()
        h = QHBoxLayout(w)
        h.setSpacing(10)
        h.addWidget(self._make_seq_library_panel(), 1)
        h.addWidget(self._make_seq_playlist_panel(), 1)
        return w

    def _make_seq_library_panel(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)

        # ── Motion Library ────────────────────────────────────────────────────
        grp_lib = QGroupBox('Motion Library')
        vl = QVBoxLayout(grp_lib)
        vl.addWidget(QLabel('~/gim8108_motions/ — double-click or Add to Playlist'))
        self.seq_lib_list = QListWidget()
        self.seq_lib_list.setMaximumHeight(130)
        self.seq_lib_list.itemDoubleClicked.connect(self._seq_add_selected)
        vl.addWidget(self.seq_lib_list)
        hLib = QHBoxLayout()
        btn_lib_refresh = QPushButton('Refresh')
        btn_lib_refresh.clicked.connect(self._seq_refresh_library)
        btn_add = QPushButton('Add to Playlist ▶')
        btn_add.clicked.connect(self._seq_add_selected)
        hLib.addWidget(btn_lib_refresh)
        hLib.addWidget(btn_add)
        vl.addLayout(hLib)
        v.addWidget(grp_lib)

        # ── Saved Sequences ───────────────────────────────────────────────────
        grp_seq = QGroupBox('Saved Sequences')
        vs = QVBoxLayout(grp_seq)
        vs.addWidget(QLabel('~/gim8108_sequences/ — double-click to load'))
        self.seq_saved_list = QListWidget()
        self.seq_saved_list.itemDoubleClicked.connect(self._seq_load_selected)
        vs.addWidget(self.seq_saved_list)

        hSeqName = QHBoxLayout()
        hSeqName.addWidget(QLabel('Name:'))
        self.seq_name_edit = QLineEdit('sequence_01')
        hSeqName.addWidget(self.seq_name_edit)
        vs.addLayout(hSeqName)

        hSeqBtn = QHBoxLayout()
        btn_seq_save = QPushButton('Save Playlist')
        btn_seq_save.clicked.connect(self._seq_save)
        btn_seq_load = QPushButton('Load')
        btn_seq_load.clicked.connect(self._seq_load_selected)
        btn_seq_del = QPushButton('Delete')
        btn_seq_del.clicked.connect(self._seq_delete_seq)
        btn_seq_ref = QPushButton('Refresh')
        btn_seq_ref.clicked.connect(self._seq_refresh_saved)
        hSeqBtn.addWidget(btn_seq_save)
        hSeqBtn.addWidget(btn_seq_load)
        hSeqBtn.addWidget(btn_seq_del)
        hSeqBtn.addWidget(btn_seq_ref)
        vs.addLayout(hSeqBtn)
        v.addWidget(grp_seq)

        self._seq_refresh_library()
        return w

    def _make_seq_playlist_panel(self):
        grp = QGroupBox('Playlist')
        v = QVBoxLayout(grp)

        self.seq_play_list = QListWidget()
        v.addWidget(self.seq_play_list)

        hEdit = QHBoxLayout()
        hEdit.addWidget(QLabel('Wait after (s):'))
        self.seq_delay_spin = QDoubleSpinBox()
        self.seq_delay_spin.setRange(0.0, 60.0)
        self.seq_delay_spin.setSingleStep(0.5)
        self.seq_delay_spin.setValue(0.5)
        self.seq_delay_spin.setFixedWidth(80)
        hEdit.addWidget(self.seq_delay_spin)
        hEdit.addStretch()
        v.addLayout(hEdit)

        hOrder = QHBoxLayout()
        for label, slot in (('Up', self._seq_item_up), ('Down', self._seq_item_down),
                             ('Remove', self._seq_item_remove), ('Clear', self._seq_clear)):
            b = QPushButton(label); b.clicked.connect(slot); hOrder.addWidget(b)
        v.addLayout(hOrder)

        sep = QFrame(); sep.setFrameShape(QFrame.HLine); sep.setFrameShadow(QFrame.Sunken)
        v.addWidget(sep)

        self.seq_status_lbl = QLabel('Stopped')
        self.seq_status_lbl.setStyleSheet('color:#555555;')
        v.addWidget(self.seq_status_lbl)

        self.seq_motion_progress = QProgressBar()
        self.seq_motion_progress.setFormat('Motion  %v / %m  frames')
        self.seq_motion_progress.setValue(0); self.seq_motion_progress.setMaximum(1)
        v.addWidget(self.seq_motion_progress)

        hTime = QHBoxLayout()
        hTime.addWidget(QLabel('Elapsed:'))
        self.seq_lbl_elapsed = QLabel('0.0 s')
        self.seq_lbl_elapsed.setFont(QFont('Monospace', 9))
        hTime.addWidget(self.seq_lbl_elapsed)
        hTime.addSpacing(16)
        hTime.addWidget(QLabel('Remaining:'))
        self.seq_lbl_remain = QLabel('---')
        self.seq_lbl_remain.setFont(QFont('Monospace', 9))
        hTime.addWidget(self.seq_lbl_remain)
        hTime.addStretch()
        v.addLayout(hTime)

        hCtrl = QHBoxLayout()
        self.btn_seq_play = QPushButton('▶  Play Sequence')
        self.btn_seq_play.setStyleSheet(
            'background:#3a3a3a;color:#dddddd;font-weight:bold;border:1px solid #777;'
        )
        self.btn_seq_play.clicked.connect(self._seq_play)
        self.btn_seq_stop = QPushButton('⏹  Stop')
        self.btn_seq_stop.setEnabled(False)
        self.btn_seq_stop.clicked.connect(self._seq_stop)
        self.chk_seq_loop = QCheckBox('Loop')
        hCtrl.addWidget(self.btn_seq_play)
        hCtrl.addWidget(self.btn_seq_stop)
        hCtrl.addWidget(self.chk_seq_loop)
        v.addLayout(hCtrl)
        return grp

    def _seq_refresh_library(self):
        self.seq_lib_list.clear()
        if not MOTION_DIR.exists():
            return
        for p in sorted(MOTION_DIR.glob('*.json')):
            self.seq_lib_list.addItem(p.stem)

    def _seq_refresh_saved(self):
        self.seq_saved_list.clear()
        if not SEQ_DIR.exists():
            return
        for p in sorted(SEQ_DIR.glob('*.json')):
            self.seq_saved_list.addItem(p.stem)

    def _seq_save(self):
        if not self._seq_items:
            QMessageBox.warning(self, 'Empty', 'Playlist is empty — nothing to save.')
            return
        raw  = self.seq_name_edit.text().strip() or 'sequence'
        name = ''.join(c if c.isalnum() or c in '-_' else '_' for c in raw)
        path = SEQ_DIR / f'{name}.json'
        if path.exists():
            if QMessageBox.question(self, 'Overwrite?', f'"{name}.json" already exists. Overwrite?',
                                    QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
                return
        data = {'name': name, 'items': list(self._seq_items)}
        path.write_text(json.dumps(data, indent=2))
        self._seq_refresh_saved()
        QMessageBox.information(self, 'Saved', f'Saved:\n{path}')

    def _seq_load_selected(self):
        item = self.seq_saved_list.currentItem()
        if not item:
            return
        path = SEQ_DIR / f'{item.text()}.json'
        try:
            data = json.loads(path.read_text())
        except Exception as exc:
            QMessageBox.warning(self, 'Load Error', str(exc))
            return
        items = data.get('items', [])
        if not items:
            QMessageBox.warning(self, 'Empty', 'Sequence file contains no items.')
            return
        # Replace current playlist
        self._seq_items = [{'name': it['name'], 'delay': float(it.get('delay', 0.0))}
                           for it in items]
        self.seq_play_list.clear()
        for it in self._seq_items:
            self.seq_play_list.addItem(f'{it["name"]}  (+{it["delay"]:.1f}s)')
        # Set name field to loaded name
        self.seq_name_edit.setText(data.get('name', item.text()))

    def _seq_delete_seq(self):
        item = self.seq_saved_list.currentItem()
        if not item:
            return
        name = item.text()
        if QMessageBox.question(self, 'Delete', f'Delete "{name}"?',
                                QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            (SEQ_DIR / f'{name}.json').unlink(missing_ok=True)
            self._seq_refresh_saved()

    def _seq_add_selected(self):
        item = self.seq_lib_list.currentItem()
        if not item:
            return
        name = item.text()
        delay = self.seq_delay_spin.value()
        self._seq_items.append({'name': name, 'delay': delay})
        self.seq_play_list.addItem(f'{name}  (+{delay:.1f}s)')

    def _seq_item_up(self):
        row = self.seq_play_list.currentRow()
        if row < 1:
            return
        self._seq_items[row - 1], self._seq_items[row] = self._seq_items[row], self._seq_items[row - 1]
        item = self.seq_play_list.takeItem(row)
        self.seq_play_list.insertItem(row - 1, item)
        self.seq_play_list.setCurrentRow(row - 1)

    def _seq_item_down(self):
        row = self.seq_play_list.currentRow()
        if row < 0 or row >= self.seq_play_list.count() - 1:
            return
        self._seq_items[row], self._seq_items[row + 1] = self._seq_items[row + 1], self._seq_items[row]
        item = self.seq_play_list.takeItem(row)
        self.seq_play_list.insertItem(row + 1, item)
        self.seq_play_list.setCurrentRow(row + 1)

    def _seq_item_remove(self):
        row = self.seq_play_list.currentRow()
        if row < 0:
            return
        self.seq_play_list.takeItem(row)
        self._seq_items.pop(row)

    def _seq_clear(self):
        self._seq_items.clear()
        self.seq_play_list.clear()

    def _seq_play(self):
        if not self._seq_items:
            QMessageBox.information(self, 'Empty playlist', 'Add motions to the playlist first.')
            return
        if not self.ch or not self.ch.is_connected:
            QMessageBox.warning(self, 'Not Connected', 'Motor driver is not connected.')
            return
        self._seq_running = True
        self._seq_idx = 0
        self._seq_start_t = time.monotonic()
        self.btn_seq_play.setEnabled(False)
        self.btn_seq_stop.setEnabled(True)
        self._seq_update_timer.start(100)
        self._seq_play_current()

    def _seq_stop(self):
        self._seq_running = False
        self._seq_delay_timer.stop()
        self._seq_update_timer.stop()
        self._play_done_callback = None
        self._stop_play()
        self.btn_seq_play.setEnabled(True)
        self.btn_seq_stop.setEnabled(False)
        self.seq_status_lbl.setText('Stopped')
        self.seq_status_lbl.setStyleSheet('color:#555555;')
        self.seq_motion_progress.setValue(0)
        self.seq_lbl_elapsed.setText('0.0 s')
        self.seq_lbl_remain.setText('---')
        self.seq_play_list.setCurrentRow(-1)

    def _seq_play_current(self):
        if not self._seq_running:
            return
        if self._seq_idx >= len(self._seq_items):
            if self.chk_seq_loop.isChecked():
                self._seq_idx = 0
            else:
                self._seq_running = False
                self._seq_update_timer.stop()
                self.btn_seq_play.setEnabled(True)
                self.btn_seq_stop.setEnabled(False)
                self.seq_status_lbl.setText('Done')
                self.seq_status_lbl.setStyleSheet('color:#888888;')
                self.seq_motion_progress.setValue(0)
                self.seq_lbl_remain.setText('0.0 s')
                self.seq_play_list.setCurrentRow(-1)
                return

        item = self._seq_items[self._seq_idx]
        name = item['name']
        self.seq_play_list.setCurrentRow(self._seq_idx)
        self.seq_status_lbl.setText(f'Playing: {name}  [{self._seq_idx + 1}/{len(self._seq_items)}]')
        self.seq_status_lbl.setStyleSheet('color:#cccccc;')

        path = MOTION_DIR / f'{name}.json'
        try:
            motion = Motion.load(path)
        except Exception as exc:
            self.seq_status_lbl.setText(f'Error loading {name}: {exc}')
            self.seq_status_lbl.setStyleSheet('color:#cc4444;')
            self._seq_idx += 1
            self._seq_play_current()
            return

        self._motion = motion
        self.seq_motion_progress.setMaximum(max(1, len(motion.frames)))
        self.seq_motion_progress.setValue(0)
        self._play_done_callback = self._seq_on_motion_done
        self._start_play()

    def _seq_on_motion_done(self):
        if not self._seq_running:
            return
        delay_ms = int(self._seq_items[self._seq_idx].get('delay', 0.0) * 1000)
        self._seq_idx += 1
        if delay_ms > 0:
            self.seq_status_lbl.setText(
                f'Waiting {self._seq_items[self._seq_idx - 1]["delay"]:.1f}s…'
            )
            self._seq_delay_timer.start(delay_ms)
        else:
            self._seq_next()

    def _seq_next(self):
        if self._seq_running:
            self._seq_play_current()

    def _seq_update_progress(self):
        if not self._seq_running:
            return
        # Update frame progress bar
        if self._motion and self._play_timer.isActive():
            self.seq_motion_progress.setValue(self._play_idx)
        # Elapsed / remaining time for the whole sequence
        elapsed = time.monotonic() - self._seq_start_t
        self.seq_lbl_elapsed.setText(f'{elapsed:.1f} s')
        # Remaining: estimate from remaining motions + delays
        remaining = 0.0
        for i in range(self._seq_idx, len(self._seq_items)):
            item = self._seq_items[i]
            path = MOTION_DIR / f'{item["name"]}.json'
            try:
                m = Motion.load(path)
                # Current motion: subtract frames already played
                if i == self._seq_idx and self._motion:
                    frames_left = max(0, len(m.frames) - self._play_idx)
                    remaining += frames_left / m.hz if m.hz > 0 else 0
                else:
                    remaining += m.duration
            except Exception:
                pass
            remaining += item.get('delay', 0.0)
        self.seq_lbl_remain.setText(f'{remaining:.1f} s')

    # ── Status graph tab ──────────────────────────────────────────────────────

    def _make_status_graph_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(4, 4, 4, 4)

        hTop = QHBoxLayout()
        self.btn_graph_clear = QPushButton('Clear')
        self.btn_graph_clear.setFixedWidth(80)
        self.btn_graph_clear.clicked.connect(self._graph_clear)
        self.lbl_graph_rate = QLabel('5 Hz')
        self.lbl_graph_rate.setStyleSheet('color:#555555;')
        hTop.addStretch()
        hTop.addWidget(QLabel('Refresh:'))
        self.cmb_graph_rate = QComboBox()
        for label, ms in (('1 Hz', 1000), ('2 Hz', 500), ('5 Hz', 200), ('10 Hz', 100)):
            self.cmb_graph_rate.addItem(label, ms)
        self.cmb_graph_rate.setCurrentIndex(2)
        self.cmb_graph_rate.currentIndexChanged.connect(self._on_graph_rate_changed)
        hTop.addWidget(self.cmb_graph_rate)
        hTop.addWidget(self.btn_graph_clear)
        v.addLayout(hTop)

        self._graph_canvas = StatusGraphCanvas()
        v.addWidget(self._graph_canvas, 1)
        return w

    def _on_graph_rate_changed(self):
        if self._graph_timer.isActive():
            ms = self.cmb_graph_rate.currentData()
            self._graph_timer.start(ms)

    def _graph_update(self):
        if not self.ch or not self.ch.is_connected or self._graph_canvas is None:
            return
        t = time.monotonic() - self._graph_t0
        self._graph_canvas.push(t, [
            self.ch.pos_abs_deg,
            self.ch.vel_out_ts,
            self.ch.current_a,
            self.ch.temperature,
            self.ch.voltage_v,
        ])
        self._graph_canvas.redraw()

    def _graph_clear(self):
        if self._graph_canvas:
            self._graph_canvas.clear()
            self._graph_t0 = time.monotonic()

    # ── Status refresh ─────────────────────────────────────────────────────────

    def _refresh_status(self):
        if not self.ch or not self.ch.is_connected:
            return
        self.lbl_pos.setText(f'Angle {self.ch.pos_abs_deg:8.1f} °')
        self.lbl_vel.setText(f'Speed    {self.ch.vel_out_ts:5.2f} t/s')
        self.lbl_cur.setText(f'Current  {self.ch.current_a:5.2f} A')
        vbus = self.ch.voltage_v
        if vbus is None:
            self.lbl_vbus.setText('Voltage  ---')
            self.lbl_vbus.setStyleSheet('color:#555555;')
        else:
            color = '#777777' if vbus < 10.0 else ('#aaaaaa' if vbus < 14.0 else '#cccccc')
            self.lbl_vbus.setText(f'Voltage  {vbus:5.1f} V')
            self.lbl_vbus.setStyleSheet(f'color:{color};')
        temp = self.ch.temperature
        if temp is None:
            self.lbl_temp.setText('Temp     ---')
            self.lbl_temp.setStyleSheet('color:#555555;')
        else:
            color = '#cc4444' if temp > 80.0 else ('#cc8844' if temp > 60.0 else '#cccccc')
            self.lbl_temp.setText(f'Temp  {temp:6.1f} °C')
            self.lbl_temp.setStyleSheet(f'color:{color};')

    def _refresh_connection(self):
        connected = self.ch is not None and self.ch.is_connected
        if connected:
            self.lbl_conn.setText('⬤  CONNECTED')
            self.lbl_conn.setStyleSheet('color:#cccccc;font-weight:bold;')
            for lbl in (self.lbl_pos, self.lbl_vel, self.lbl_cur, self.lbl_vbus, self.lbl_temp):
                lbl.setStyleSheet('color:#cccccc;')
            sn = self.ch.serial_number if self.ch else ''
            if sn:
                self.lbl_serial.setText(f'SN: {sn}')
                self.lbl_serial.setStyleSheet('color:#555555;')
            # Start graph timer if not already running
            if not self._graph_timer.isActive():
                self._graph_t0 = time.monotonic()
                self._graph_timer.start(self.cmb_graph_rate.currentData())
        else:
            self.lbl_conn.setText('⬤  DISCONNECTED')
            self.lbl_conn.setStyleSheet('color:#444444;font-weight:bold;')
            self.lbl_pos.setText('Angle    ---')
            self.lbl_vel.setText('Speed    ---')
            self.lbl_cur.setText('Current  ---')
            self.lbl_vbus.setText('Voltage  ---')
            self.lbl_temp.setText('Temp     ---')
            for lbl in (self.lbl_pos, self.lbl_vel, self.lbl_cur, self.lbl_vbus, self.lbl_temp):
                lbl.setStyleSheet('color:#555555;')
            self.lbl_serial.setText('')
            self._stop_play()
            self._graph_timer.stop()

    def _on_calib_msg(self, msg: str):
        self.calib_log.append(msg)

    def _on_serial_update(self, sn: str):
        self.lbl_serial.setText(f'SN: {sn}')
        self.lbl_serial.setStyleSheet('color:#555555;')

    def _on_config_update(self, key: str, value: float):
        def _set(w, v):
            w.blockSignals(True); w.setValue(v); w.blockSignals(False)

        if   key == 'pos_gain':    _set(self.spin_pos_gain, value)
        elif key == 'vel_gain':    _set(self.spin_vel_gain, value)
        elif key == 'vel_int':     _set(self.spin_vel_int,  value)
        elif key == 'traj_vel':    _set(self.slider_spd, int(round(value * 10)))
        elif key == 'traj_accel':  _set(self.spin_accel, value)
        elif key == 'traj_decel':  _set(self.spin_decel, value)
        elif key == 'cur_limit':
            _set(self.slider_cur, int(round(value * 10)))
            self.lbl_cur_set.setText(f'{value:.1f} A')


# ── Main window ───────────────────────────────────────────────────────────────

class MultiMotorGUI(QMainWindow):
    def __init__(self, node: MultiGuiNode):
        super().__init__()
        self.node = node
        self._build()
        self.setWindowTitle('GIM8108-8 Motor Control')
        self.setMinimumSize(960, 640)

    def _build(self):
        central = QWidget()
        vroot = QVBoxLayout(central)
        vroot.setContentsMargins(0, 0, 0, 0)
        vroot.setSpacing(0)

        # ── Global E-STOP bar ──────────────────────────────────────────────
        bar = QWidget()
        bar.setFixedHeight(44)
        bar.setStyleSheet('background:#1e1e1e;')
        bh = QHBoxLayout(bar)
        bh.setContentsMargins(14, 6, 14, 6)
        lbl_title = QLabel('GIM8108-8  Motor Control')
        lbl_title.setStyleSheet('color:#444444; font-size:12px;')
        self.btn_estop_all = QPushButton('⚠  ALL E-STOP')
        self.btn_estop_all.setFixedSize(160, 32)
        self.btn_estop_all.setStyleSheet(
            'QPushButton { background:#883333; color:#ffffff; font-weight:bold; '
            'font-size:12px; border:none; border-radius:3px; }'
            'QPushButton:hover   { background:#aa4444; }'
            'QPushButton:pressed { background:#662222; }'
        )
        self.btn_estop_all.clicked.connect(self._on_estop_all)
        bh.addWidget(lbl_title)
        bh.addStretch()
        bh.addWidget(self.btn_estop_all)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        self.list_panel   = MotorListPanel(self.node)
        self.detail_panel = MotorDetailPanel()

        self.list_panel.motor_selected.connect(self._on_motor_selected)

        splitter.addWidget(self.list_panel)
        splitter.addWidget(self.detail_panel)
        splitter.setSizes([220, 740])

        vroot.addWidget(bar)
        vroot.addWidget(sep)
        vroot.addWidget(splitter, 1)
        self.setCentralWidget(central)

    def _on_estop_all(self):
        for ch in self.node.channels.values():
            ch.call_estop()

    def add_default_motor(self, name: str = 'Motor 0', topic_ns: str = DEFAULT_NS):
        panel_settings = self.list_panel._load_panel_settings()
        ch = MotorChannel(name=name, topic_ns=topic_ns,
                          gui_settings=panel_settings.get(topic_ns, {}))
        self.node.add_channel(ch)
        self.list_panel.add_card(ch)
        self.detail_panel.set_channel(ch)

    def _on_motor_selected(self, ns: str):
        # Save current motor's panel settings before switching
        if self.detail_panel.ch:
            self.detail_panel._save_settings_to_channel(self.detail_panel.ch)
            self.list_panel.save_panel_settings()
        if not ns:
            self.detail_panel.set_channel(None)
            return
        ch = self.node.channels.get(ns)
        if ch:
            self.detail_panel.set_channel(ch)

    def closeEvent(self, event):
        if self.detail_panel.ch:
            self.detail_panel._save_settings_to_channel(self.detail_panel.ch)
        self.list_panel.save_panel_settings()
        event.accept()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MultiGuiNode()

    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLE)

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    window = MultiMotorGUI(node)
    window.show()
    # Defer ROS2 subscription creation until after the Qt event loop starts,
    # to avoid race conditions with the rclpy.spin() background thread.
    QTimer.singleShot(0, lambda: (
        window.add_default_motor(),
        window.list_panel._load_and_relaunch(),
    ))

    exit_code = app.exec_()

    # Terminate any GUI-launched processes
    for ch in node.channels.values():
        if ch.process:
            ch.process.terminate()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
