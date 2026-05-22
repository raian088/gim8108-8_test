"""
GIM8108-8 Motion Recorder & Player
Records position sequences and plays them back via cmd_pos_deg.
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
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem,
    QMainWindow, QMessageBox, QProgressBar, QPushButton,
    QVBoxLayout, QWidget,
)

MOTION_DIR = Path.home() / 'gim8108_motions'
DRIVER_NS  = '/gim8108_motor_node'

_QOS_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


# ── Data model ────────────────────────────────────────────────────────────────

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
            {'name': self.name, 'hz': self.hz, 'frames': self.frames},
            indent=2
        ))

    @classmethod
    def load(cls, path: Path) -> 'Motion':
        d = json.loads(path.read_text())
        return cls(name=d['name'], hz=float(d['hz']), frames=list(d['frames']))


# ── ROS node ──────────────────────────────────────────────────────────────────

class MotionNode(Node):
    def __init__(self):
        super().__init__('gim8108_motion_gui')

        self.pos_deg      = 0.0
        self.is_connected = False
        self.on_state     = None
        self.on_connected = None

        self.pub_pos    = self.create_publisher(Float64, f'{DRIVER_NS}/cmd_pos_deg', 10)
        self.enable_cli = self.create_client(SetBool, f'{DRIVER_NS}/enable')

        self.create_subscription(JointState, f'{DRIVER_NS}/joint_state',
                                 self._on_state, 10)
        self.create_subscription(Bool, f'{DRIVER_NS}/is_connected',
                                 self._on_connected, _QOS_LATCHED)

    def _on_state(self, msg: JointState):
        if msg.position:
            self.pos_deg = math.degrees(msg.position[0]) % 360.0
        if self.on_state:
            self.on_state()

    def _on_connected(self, msg: Bool):
        self.is_connected = msg.data
        if self.on_connected:
            self.on_connected()

    def send_pos(self, deg: float):
        m = Float64(); m.data = float(deg)
        self.pub_pos.publish(m)

    def call_enable(self, enable: bool):
        if not self.enable_cli.service_is_ready():
            return
        req = SetBool.Request(); req.data = enable
        self.enable_cli.call_async(req)


# ── GUI ───────────────────────────────────────────────────────────────────────

class MotionGUI(QMainWindow):
    _sig_state     = pyqtSignal()
    _sig_connected = pyqtSignal()

    def __init__(self, node: MotionNode):
        super().__init__()
        self.node = node
        self.node.on_state     = self._sig_state.emit
        self.node.on_connected = self._sig_connected.emit
        self._sig_state.connect(self._refresh_status)
        self._sig_connected.connect(self._refresh_connection)

        # Recording state
        self._recorded: list = []
        self._record_hz: float = 10.0
        self._record_timer = QTimer()
        self._record_timer.timeout.connect(self._record_frame)

        # Playback state
        self._motion: Optional[Motion] = None
        self._play_idx: int = 0
        self._play_timer = QTimer()
        self._play_timer.timeout.connect(self._play_step)

        MOTION_DIR.mkdir(exist_ok=True)
        self._build_ui()
        self.setWindowTitle('GIM8108-8 Motion Player')
        self.setMinimumWidth(720)
        self._refresh_list()

    # ── UI build ──────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        vbox = QVBoxLayout(root)
        vbox.addWidget(self._build_status())
        hbox = QHBoxLayout()
        hbox.addWidget(self._build_record(), 1)
        hbox.addWidget(self._build_play(), 1)
        vbox.addLayout(hbox)

    def _build_status(self):
        grp = QGroupBox('Status')
        h = QHBoxLayout(grp)
        self.lbl_conn = QLabel('Disconnected')
        self.lbl_conn.setFont(QFont('', 10, QFont.Bold))
        self.lbl_conn.setStyleSheet('color:red;')
        self.lbl_conn.setMinimumWidth(120)
        self.lbl_pos = QLabel('Angle:  ---')
        self.lbl_pos.setFont(QFont('Monospace', 9))
        self.lbl_pos.setMinimumWidth(150)
        h.addWidget(self.lbl_conn)
        h.addWidget(self.lbl_pos)
        h.addStretch()
        return grp

    def _build_record(self):
        grp = QGroupBox('Record')
        v = QVBoxLayout(grp)

        # Hz
        hHz = QHBoxLayout()
        hHz.addWidget(QLabel('Capture Hz:'))
        self.cmb_hz = QComboBox()
        for hz in (5, 10, 20, 30):
            self.cmb_hz.addItem(f'{hz} Hz', float(hz))
        self.cmb_hz.setCurrentIndex(1)   # default 10 Hz
        hHz.addWidget(self.cmb_hz)
        hHz.addStretch()
        v.addLayout(hHz)

        self.lbl_frames = QLabel('Frames: 0  (0.0 s)')
        v.addWidget(self.lbl_frames)

        # Record / Stop
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

        btn_save = QPushButton('Save Motion')
        btn_save.clicked.connect(self._save_motion)
        v.addWidget(btn_save)

        v.addStretch()
        return grp

    def _build_play(self):
        grp = QGroupBox('Playback')
        v = QVBoxLayout(grp)

        v.addWidget(QLabel('Saved motions  (~/gim8108_motions/):'))
        self.lst = QListWidget()
        self.lst.setMaximumHeight(120)
        self.lst.itemSelectionChanged.connect(self._on_motion_selected)
        v.addWidget(self.lst)

        hListBtn = QHBoxLayout()
        btn_refresh = QPushButton('Refresh')
        btn_refresh.clicked.connect(self._refresh_list)
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

        # Speed + Loop
        hSpd = QHBoxLayout()
        hSpd.addWidget(QLabel('Speed:'))
        self.cmb_speed = QComboBox()
        for label, val in (('0.25x', 0.25), ('0.5x', 0.5),
                            ('1x', 1.0), ('2x', 2.0), ('4x', 4.0)):
            self.cmb_speed.addItem(label, val)
        self.cmb_speed.setCurrentIndex(2)   # 1x
        hSpd.addWidget(self.cmb_speed)
        self.chk_loop = QCheckBox('Loop')
        hSpd.addWidget(self.chk_loop)
        hSpd.addStretch()
        v.addLayout(hSpd)

        # Play / Stop
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

    # ── Record logic ──────────────────────────────────────────────────────────

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
        self._refresh_list()
        QMessageBox.information(self, 'Saved', f'Saved:\n{path}')

    # ── Playback logic ────────────────────────────────────────────────────────

    def _refresh_list(self):
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
        self.lbl_info.setText(
            f'{m.name}   {n} frames @ {m.hz} Hz   ({m.duration:.1f} s)'
        )
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
            self._refresh_list()
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
        # Auto-enable motor
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
        self.node.send_pos(self._motion.frames[self._play_idx])
        self.progress.setValue(self._play_idx + 1)
        self._play_idx += 1

    def _stop_play(self):
        self._play_timer.stop()
        self.btn_play.setEnabled(self._motion is not None)
        self.btn_stop_play.setEnabled(False)

    # ── Status ────────────────────────────────────────────────────────────────

    def _refresh_status(self):
        if self.node.is_connected:
            self.lbl_pos.setText(f'Angle:  {self.node.pos_deg:6.1f} °')

    def _refresh_connection(self):
        if self.node.is_connected:
            self.lbl_conn.setText('Connected')
            self.lbl_conn.setStyleSheet('color:green;font-weight:bold;')
        else:
            self.lbl_conn.setText('Disconnected')
            self.lbl_conn.setStyleSheet('color:red;font-weight:bold;')
            self.lbl_pos.setText('Angle:  ---')
            self._stop_play()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()

    app = QApplication(sys.argv)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    window = MotionGUI(node)
    window.show()
    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
