"""
GIM8108-8 Multi-Motor Control GUI
Left  : motor list (status cards + Add)
Right : selected motor — Motor Control tab | Motion tab
"""

import json
import math
import subprocess
import sys
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

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

MOTION_DIR  = Path.home() / 'gim8108_motions'
DEFAULT_NS  = '/gim8108_motor_node'   # backward-compatible with start.launch.py

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
QScrollBar::handle:vertical, QScrollBar::handle:horizontal {
    background:#404040; border-radius:3px; min-length:20px;
}
QScrollBar::handle:vertical:hover,
QScrollBar::handle:horizontal:hover { background:#606060; }
QScrollBar::add-line, QScrollBar::sub-line { height:0; width:0; }

QFrame[frameShape="4"], QFrame[frameShape="5"] { color:#333333; }
"""

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

    # State
    pos_deg: float = 0.0
    pos_abs_deg: float = 0.0
    vel_out_ts: float = 0.0
    current_a: float = 0.0
    voltage_v: Optional[float] = None
    is_connected: bool = False
    serial_number: str = ''
    _cfg: dict = field(default_factory=dict)

    # Callbacks (set by the detail panel)
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
        if ch.on_state: ch.on_state()

    def _on_voltage(self, msg: Float64, ch: MotorChannel):
        ch.voltage_v = msg.data
        if ch.on_state: ch.on_state()

    def _on_connected(self, msg: Bool, ch: MotorChannel):
        if ch.is_connected != msg.data:
            ch.is_connected = msg.data
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
        try:
            result = subprocess.run(
                ['python3', '-c',
                 'import odrive, sys\n'
                 'devs = []\n'
                 'try:\n'
                 '  import usb.core\n'
                 '  for d in usb.core.find(idVendor=0x1209,idProduct=0x0d32,find_all=True):\n'
                 '    try: devs.append(usb.util.get_string(d,d.iSerialNumber))\n'
                 '    except: pass\n'
                 'except Exception:\n'
                 '  pass\n'
                 'if not devs:\n'
                 '  import odrive\n'
                 '  odrv = odrive.find_any(timeout=3)\n'
                 '  devs.append(hex(odrv.serial_number)[2:].upper())\n'
                 'print("\\n".join(devs))'],
                capture_output=True, text=True, timeout=12
            )
            serials = [s.strip() for s in result.stdout.splitlines() if s.strip()]
        except Exception:
            serials = []
        self._scan_done.emit(serials)

    def _on_scan_done(self, serials: list):
        self.btn_scan.setEnabled(True)
        self.btn_scan.setText('🔍 Scan')
        if not serials:
            QMessageBox.information(self, 'Not found',
                'No ODrive detected.\nMake sure it is connected:\n  usbipd attach --wsl --busid <BUSID>')
            return
        current = self.serial_combo.lineEdit().text()
        self.serial_combo.clear()
        self.serial_combo.addItems(serials)
        if current and current not in serials:
            self.serial_combo.lineEdit().setText(current)

    @property
    def motor_name(self): return self.name_edit.text().strip() or 'Motor'
    @property
    def axis(self): return self.axis_combo.currentIndex()
    @property
    def gear_ratio(self): return self.gear_spin.value()
    @property
    def serial_number(self): return self.serial_combo.currentText().strip()


# ── Motor card (left panel) ───────────────────────────────────────────────────

class MotorCard(QWidget):
    clicked = pyqtSignal()
    remove_requested = pyqtSignal()

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
        btn_del.clicked.connect(self.remove_requested.emit)

        layout.addWidget(self.dot)
        layout.addLayout(info)
        layout.addStretch()
        layout.addWidget(btn_del)

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
        self.setFixedWidth(220)
        self._build()

    def _build(self):
        v = QVBoxLayout(self)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(4)

        header = QLabel('MOTORS')
        header.setStyleSheet('color:#555555; font-size:10px; font-weight:bold; letter-spacing:2px;')
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
        dlg = AddMotorDialog(self, len(self.cards))
        if dlg.exec_() != QDialog.Accepted:
            return

        idx   = len(self.cards)
        rname = f'motor_{idx}'
        ns    = f'/motor_{idx}/gim8108_motor_node'
        name  = dlg.motor_name
        axis  = dlg.axis
        gr    = dlg.gear_ratio

        # Launch motor node in new namespace
        cmd = [
            'ros2', 'run', 'gim8108_driver', 'gim8108_motor_node_usb',
            '--ros-args',
            '--remap', f'__ns:=/motor_{idx}',
            '-p', f'axis:={axis}',
            '-p', f'gear_ratio:={gr}',
            '-p', 'home_on_connect:=true',
        ]
        if dlg.serial_number:
            cmd += ['-p', f'serial_number:={dlg.serial_number}']
        try:
            proc = subprocess.Popen(cmd)
        except FileNotFoundError:
            QMessageBox.critical(self, 'Error', 'ros2 not found. Make sure ROS2 is sourced.')
            return

        ch = MotorChannel(name=name, topic_ns=ns, process=proc)
        self.node.add_channel(ch)
        self.add_card(ch)

        # Wire card refresh to the channel's state callbacks
        card = self.cards[ns]
        ch.on_connected = lambda c=card, n=ns: (c.refresh(), self.motor_selected.emit(n)
                                                 if self._selected_ns == n else None)
        ch.on_state = lambda c=card: c.refresh()

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
        ch.on_state = None
        ch.on_connected = None
        ch.on_calib = None
        ch.on_config = None
        ch.on_serial = None

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

        MOTION_DIR.mkdir(exist_ok=True)

        self._sig_state.connect(self._refresh_status)
        self._sig_connected.connect(self._refresh_connection)
        self._sig_calib.connect(self._on_calib_msg)
        self._sig_config.connect(self._on_config_update)
        self._sig_serial.connect(self._on_serial_update)

        self._build()
        self._refresh_motion_list()

    # ── Channel switching ─────────────────────────────────────────────────────

    def set_channel(self, ch: MotorChannel):
        if self.ch:
            self.ch.on_state     = None
            self.ch.on_connected = None
            self.ch.on_calib     = None
            self.ch.on_config    = None
        self.ch = ch
        if ch:
            ch.on_state     = self._sig_state.emit
            ch.on_connected = self._sig_connected.emit
            ch.on_calib     = self._sig_calib.emit
            ch.on_config    = self._sig_config.emit
            ch.on_serial    = self._sig_serial.emit
        self._refresh_connection()
        self._refresh_status()

    # ── UI build ──────────────────────────────────────────────────────────────

    def _build(self):
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        v.addWidget(self._make_status_bar())
        tabs = QTabWidget()
        tabs.addTab(self._make_control_tab(), 'Motor Control')
        tabs.addTab(self._make_motion_tab(),  'Motion')
        v.addWidget(tabs)

    # ── Status bar ────────────────────────────────────────────────────────────

    def _make_status_bar(self):
        grp = QGroupBox('Status')
        h = QHBoxLayout(grp)
        h.setSpacing(20)

        self.lbl_conn = QLabel('⬤  DISCONNECTED')
        self.lbl_conn.setFont(QFont('', 11, QFont.Bold))
        self.lbl_conn.setStyleSheet('color:#444444; letter-spacing:2px;')
        self.lbl_conn.setMinimumWidth(160)

        sep = QFrame(); sep.setFrameShape(QFrame.VLine)

        self.lbl_pos    = QLabel('Angle    ---')
        self.lbl_vel    = QLabel('Speed    ---')
        self.lbl_cur    = QLabel('Current  ---')
        self.lbl_vbus   = QLabel('Voltage  ---')
        self.lbl_serial = QLabel('')
        mono = QFont('Monospace', 10)
        for lbl in (self.lbl_pos, self.lbl_vel, self.lbl_cur, self.lbl_vbus):
            lbl.setFont(mono)
            lbl.setMinimumWidth(155)
            lbl.setStyleSheet('color:#555555;')
        self.lbl_serial.setFont(QFont('Monospace', 9))
        self.lbl_serial.setStyleSheet('color:#3a3a3a;')

        h.addWidget(self.lbl_conn)
        h.addWidget(sep)
        for lbl in (self.lbl_pos, self.lbl_vel, self.lbl_cur, self.lbl_vbus):
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
        self.spin_pos.setSuffix(' °')
        self.spin_pos.setFixedWidth(100)
        self.spin_pos.valueChanged.connect(self._on_pos_spin)
        h.addWidget(self.slider_pos)
        h.addWidget(self.spin_pos)
        v.addLayout(h)
        row = QHBoxLayout()
        for t in ('0°', '90°', '180°', '270°', '360°'):
            row.addWidget(QLabel(t))
            if t != '360°': row.addStretch()
        v.addLayout(row)
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
                QTimer.singleShot(150, lambda: self.ch and self.ch.send_pos_deg(self.ch.pos_deg))

        for btn, m in ((self.btn_mode_pos, CTRL_POSITION),
                       (self.btn_mode_vel, CTRL_VELOCITY),
                       (self.btn_mode_trq, CTRL_TORQUE)):
            btn.blockSignals(True)
            btn.setChecked(mode == m)
            btn.blockSignals(False)

        self.slider_pos.setEnabled(mode == CTRL_POSITION)
        self.spin_pos.setEnabled(mode == CTRL_POSITION)
        self.slider_spd.setEnabled(mode != CTRL_TORQUE)

    def _on_pos_slider(self, val: int):
        deg = val / 10.0
        self.spin_pos.blockSignals(True)
        self.spin_pos.setValue(deg)
        self.spin_pos.blockSignals(False)
        if self.ch and self._current_mode == CTRL_POSITION:
            self.ch.send_pos_deg(deg)

    def _on_pos_spin(self, val: float):
        self.slider_pos.blockSignals(True)
        self.slider_pos.setValue(int(val * 10))
        self.slider_pos.blockSignals(False)
        if self.ch and self._current_mode == CTRL_POSITION:
            self.ch.send_pos_deg(val)

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
        target = self.ch.pos_abs_deg - frac if frac <= 180.0 else self.ch.pos_abs_deg + (360.0 - frac)
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
            self._recorded.append(self.ch.pos_deg)
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

    # ── Status refresh ─────────────────────────────────────────────────────────

    def _refresh_status(self):
        if not self.ch or not self.ch.is_connected:
            return
        pos = self.ch.pos_deg
        if pos >= 360.0: pos -= 360.0
        self.lbl_pos.setText(f'Angle    {pos:6.1f} °')
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

    def _refresh_connection(self):
        connected = self.ch is not None and self.ch.is_connected
        if connected:
            self.lbl_conn.setText('⬤  CONNECTED')
            self.lbl_conn.setStyleSheet('color:#cccccc;font-weight:bold;letter-spacing:2px;')
            for lbl in (self.lbl_pos, self.lbl_vel, self.lbl_cur, self.lbl_vbus):
                lbl.setStyleSheet('color:#cccccc;')
            sn = self.ch.serial_number if self.ch else ''
            if sn:
                self.lbl_serial.setText(f'SN: {sn}')
                self.lbl_serial.setStyleSheet('color:#555555;')
        else:
            self.lbl_conn.setText('⬤  DISCONNECTED')
            self.lbl_conn.setStyleSheet('color:#444444;font-weight:bold;letter-spacing:2px;')
            self.lbl_pos.setText('Angle    ---')
            self.lbl_vel.setText('Speed    ---')
            self.lbl_cur.setText('Current  ---')
            self.lbl_vbus.setText('Voltage  ---')
            for lbl in (self.lbl_pos, self.lbl_vel, self.lbl_cur, self.lbl_vbus):
                lbl.setStyleSheet('color:#555555;')
            self.lbl_serial.setText('')
            self._stop_play()

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
        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        self.list_panel   = MotorListPanel(self.node)
        self.detail_panel = MotorDetailPanel()

        self.list_panel.motor_selected.connect(self._on_motor_selected)

        splitter.addWidget(self.list_panel)
        splitter.addWidget(self.detail_panel)
        splitter.setSizes([220, 740])
        self.setCentralWidget(splitter)

    def add_default_motor(self, name: str = 'Motor 0', topic_ns: str = DEFAULT_NS):
        ch = MotorChannel(name=name, topic_ns=topic_ns)
        self.node.add_channel(ch)
        self.list_panel.add_card(ch)

        # Wire card refresh
        card = self.list_panel.cards[topic_ns]
        ch.on_connected = lambda c=card, ns=topic_ns: (
            c.refresh(),
            self._on_motor_selected(ns) if self.list_panel._selected_ns == ns else None
        )
        ch.on_state = lambda c=card: c.refresh()

        # Wire detail panel (first motor selected by default)
        self.detail_panel.set_channel(ch)

    def _on_motor_selected(self, ns: str):
        if not ns:
            self.detail_panel.set_channel(None)
            return
        ch = self.node.channels.get(ns)
        if ch:
            self.detail_panel.set_channel(ch)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MultiGuiNode()

    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLE)

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    window = MultiMotorGUI(node)
    window.add_default_motor()
    window.show()

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
