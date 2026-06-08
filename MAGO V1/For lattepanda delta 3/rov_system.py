#!/usr/bin/env python3
"""
============================================================
 MAGO V1 - PC2 Control Station (PyQt5 GUI)
 Deep Sea Exploration System
 by Sohayb Shaaben
============================================================

 Professional ROV control interface with:
 - Live video feed (UDP)
 - 2D Gyro Compass (QPainter, NO OpenGL)
 - 2D Sonar display (pyqtgraph)
 - Sensor data panel
 - Movement controls (keyboard)
 - Mission Start/Stop with auto-record
 - Manual recording and photo capture

 Communication:
   TCP Port 5005: Bidirectional text data
   UDP Port 5007: Receive video stream
"""

import sys
import os
import socket
import threading
import time
import re
import struct
import platform
import subprocess
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QSlider, QFrame, QSizePolicy,
    QDialog, QLineEdit, QFormLayout, QDialogButtonBox, QSplashScreen,
    QGroupBox, QSpacerItem, QCheckBox
)
from PyQt5.QtCore import (
    Qt, QTimer, QElapsedTimer, QRectF, QPointF, QSize, pyqtSignal, QObject
)
from PyQt5.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QRadialGradient,
    QLinearGradient, QPixmap, QImage, QIcon, QKeySequence, QPainterPath
)

import pyqtgraph as pg
# CRITICAL: Disable OpenGL to prevent LattePanda graphics crash
pg.setConfigOptions(useOpenGL=False, antialias=False)

import numpy as np

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

# ─────────────── Configuration ───────────────
TCP_PORT = 5005
UDP_VIDEO_PORT = 5007
DEFAULT_PC1_IP = "192.168.1.10"
WINDOW_WIDTH = 1920
WINDOW_HEIGHT = 1080

# ─────────────── Directories ───────────────
PC_NAME = platform.node() or "UNKNOWN_PC"
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Data")
VIDEO_DIR = os.path.join(DATA_DIR, "Videos", PC_NAME)
PHOTO_DIR = os.path.join(DATA_DIR, "Photos", PC_NAME)

for d in [VIDEO_DIR, PHOTO_DIR]:
    os.makedirs(d, exist_ok=True)


# ═══════════════════════════════════════════════════════════
#  Signal Bridge for Thread-Safe GUI Updates
# ═══════════════════════════════════════════════════════════
class SignalBridge(QObject):
    data_received = pyqtSignal(str)
    frame_received = pyqtSignal(bytes)
    connection_changed = pyqtSignal(bool)
    status_message = pyqtSignal(str)


# ═══════════════════════════════════════════════════════════
#  Network Client
# ═══════════════════════════════════════════════════════════
class NetworkClient:
    """Handles TCP and UDP connections to PC1."""

    def __init__(self, bridge):
        self.bridge = bridge
        self.tcp_socket = None
        self.udp_socket = None
        self.connected = False
        self.running = True
        self.pc1_ip = DEFAULT_PC1_IP
        self.lock = threading.Lock()

    def connect(self, pc1_ip):
        """Connect to PC1 via TCP and start UDP listener."""
        self.pc1_ip = pc1_ip
        try:
            # TCP Connection
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.settimeout(5.0)
            self.tcp_socket.connect((pc1_ip, TCP_PORT))
            self.tcp_socket.settimeout(0.5)
            self.connected = True
            self.bridge.connection_changed.emit(True)
            self.bridge.status_message.emit(f"Connected to PC1 at {pc1_ip}")

            # Start TCP reader thread
            t = threading.Thread(target=self._tcp_reader, daemon=True)
            t.start()

            # Start UDP listener
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536 * 4)
            self.udp_socket.bind(("0.0.0.0", UDP_VIDEO_PORT))
            self.udp_socket.settimeout(1.0)

            t2 = threading.Thread(target=self._udp_reader, daemon=True)
            t2.start()

        except Exception as e:
            self.connected = False
            self.bridge.connection_changed.emit(False)
            self.bridge.status_message.emit(f"Connection failed: {e}")

    def disconnect(self):
        """Disconnect from PC1."""
        self.connected = False
        self.running = False
        with self.lock:
            if self.tcp_socket:
                try:
                    self.tcp_socket.close()
                except Exception:
                    pass
                self.tcp_socket = None
            if self.udp_socket:
                try:
                    self.udp_socket.close()
                except Exception:
                    pass
                self.udp_socket = None
        self.bridge.connection_changed.emit(False)

    def send_command(self, cmd):
        """Send a command to PC1 via TCP."""
        with self.lock:
            if self.tcp_socket and self.connected:
                try:
                    self.tcp_socket.sendall((cmd + "\n").encode('utf-8'))
                except Exception as e:
                    self.bridge.status_message.emit(f"Send error: {e}")
                    self.connected = False
                    self.bridge.connection_changed.emit(False)

    def _tcp_reader(self):
        """Read TCP data from PC1 (sensor data)."""
        buffer = ""
        while self.running and self.connected:
            try:
                data = self.tcp_socket.recv(4096)
                if not data:
                    self.bridge.status_message.emit("PC1 disconnected")
                    self.connected = False
                    self.bridge.connection_changed.emit(False)
                    break

                buffer += data.decode('utf-8', errors='replace')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.bridge.data_received.emit(line)

            except socket.timeout:
                continue
            except Exception as e:
                if self.running and self.connected:
                    self.bridge.status_message.emit(f"TCP read error: {e}")
                    self.connected = False
                    self.bridge.connection_changed.emit(False)
                break

    def _udp_reader(self):
        """Read UDP video frames from PC1."""
        while self.running and self.connected:
            try:
                data, addr = self.udp_socket.recvfrom(65536)
                if data:
                    self.bridge.frame_received.emit(data)
            except socket.timeout:
                continue
            except Exception:
                if self.running:
                    continue
                break


# ═══════════════════════════════════════════════════════════
#  2D Compass Widget (QPainter, NO OpenGL)
# ═══════════════════════════════════════════════════════════
class CompassWidget(QWidget):
    """CPU-rendered 2D compass widget showing yaw orientation."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.setMinimumSize(200, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def setOrientation(self, yaw, roll, pitch):
        self.yaw = yaw
        self.roll = roll
        self.pitch = pitch
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        size = min(w, h) - 10
        cx = w / 2
        cy = h / 2
        radius = size / 2 - 5

        # Background
        bg_grad = QRadialGradient(cx, cy, radius)
        bg_grad.setColorAt(0, QColor(20, 30, 50))
        bg_grad.setColorAt(1, QColor(10, 15, 25))
        painter.setBrush(QBrush(bg_grad))
        painter.setPen(QPen(QColor(40, 80, 140), 2))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        # Concentric circles
        for r_frac in [0.25, 0.5, 0.75]:
            r = radius * r_frac
            painter.setPen(QPen(QColor(30, 60, 100, 80), 1, Qt.DotLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QPointF(cx, cy), r, r)

        # Compass markings
        painter.setPen(QPen(QColor(150, 200, 255), 1))
        font = QFont("Consolas", max(8, int(radius * 0.12)))
        font.setBold(True)
        painter.setFont(font)

        labels = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        angles_deg = [0, 45, 90, 135, 180, 225, 270, 315]

        for i, (label, angle) in enumerate(zip(labels, angles_deg)):
            rad = (angle - 90) * 3.14159 / 180.0
            # Tick mark
            inner_r = radius * 0.88
            outer_r = radius * 0.95
            x1 = cx + inner_r * np.cos(rad)
            y1 = cy + inner_r * np.sin(rad)
            x2 = cx + outer_r * np.cos(rad)
            y2 = cy + outer_r * np.sin(rad)
            pen_width = 2 if i % 2 == 0 else 1
            painter.setPen(QPen(QColor(100, 180, 255), pen_width))
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

            # Label
            if i % 2 == 0:  # Only cardinal directions
                text_r = radius * 0.75
                tx = cx + text_r * np.cos(rad)
                ty = cy + text_r * np.sin(rad)
                color = QColor(255, 100, 100) if label == "N" else QColor(150, 200, 255)
                painter.setPen(QPen(color))
                painter.drawText(QRectF(tx - 12, ty - 8, 24, 16), Qt.AlignCenter, label)

        # Yaw needle (rotates with yaw)
        yaw_rad = (self.yaw - 90) * 3.14159 / 180.0

        # North pointer (red triangle)
        needle_len = radius * 0.65
        needle_w = radius * 0.08

        # Triangle points
        tip_x = cx + needle_len * np.cos(yaw_rad)
        tip_y = cy + needle_len * np.sin(yaw_rad)

        perp_rad = yaw_rad + np.pi / 2
        base1_x = cx + needle_w * np.cos(perp_rad)
        base1_y = cy + needle_w * np.sin(perp_rad)
        base2_x = cx - needle_w * np.cos(perp_rad)
        base2_y = cy - needle_w * np.sin(perp_rad)

        # Tail (opposite direction)
        tail_len = radius * 0.3
        tail_x = cx - tail_len * np.cos(yaw_rad)
        tail_y = cy - tail_len * np.sin(yaw_rad)

        path = QPainterPath()
        path.moveTo(tip_x, tip_y)
        path.lineTo(base1_x, base1_y)
        path.lineTo(tail_x, tail_y)
        path.lineTo(base2_x, base2_y)
        path.closeSubpath()

        # Fill top half red, bottom half blue
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.setBrush(QBrush(QColor(255, 60, 60)))
        painter.drawPath(path)

        # Center dot
        painter.setBrush(QBrush(QColor(200, 200, 200)))
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.drawEllipse(QPointF(cx, cy), 4, 4)

        # Yaw text at bottom of widget
        painter.setPen(QPen(QColor(180, 220, 255)))
        font2 = QFont("Consolas", max(9, int(radius * 0.1)))
        painter.setFont(font2)
        painter.drawText(QRectF(cx - radius, cy + radius + 2, radius * 2, 20),
                         Qt.AlignCenter, f"Yaw: {self.yaw:.1f}°")

        painter.end()


# ═══════════════════════════════════════════════════════════
#  2D Sonar Widget (pyqtgraph)
# ═══════════════════════════════════════════════════════════
class SonarWidget(pg.PlotWidget):
    """2D sonar display with concentric circles, beam, and target dot."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Configure plot
        self.setAspectLocked(True)
        self.hideAxis('bottom')
        self.hideAxis('left')
        self.setBackground(QColor(10, 15, 25))

        # Range in cm
        self.max_range = 400.0

        # Draw concentric circles
        for r in range(50, int(self.max_range) + 1, 50):
            circle = pg.CircleROI((0, 0), (r * 2, r * 2), movable=False, pen=pg.mkPen(
                QColor(30, 80, 60, 120), width=1, style=Qt.DotLine))
            self.addItem(circle)

        # Range labels
        for r in [100, 200, 300, 400]:
            text = pg.TextItem(f"{r}cm", color=QColor(60, 140, 100, 180), anchor=(0, 1))
            text.setPos(r, 0)
            self.addItem(text)

        # Beam line (center outward)
        self.beam_line = pg.PlotCurveItem(pen=pg.mkPen(QColor(0, 255, 100, 150), width=2))
        self.addItem(self.beam_line)

        # Target dot
        self.target_dot = pg.ScatterPlotItem(
            size=12, pen=pg.mkPen(QColor(255, 50, 50), width=2),
            brush=pg.mkBrush(QColor(255, 50, 50, 180))
        )
        self.addItem(self.target_dot)

        # Sweep line (rotating)
        self.sweep_line = pg.PlotCurveItem(pen=pg.mkPen(QColor(0, 200, 100, 80), width=1))
        self.addItem(self.sweep_line)

        self.sweep_angle = 0
        self.target_distance = 0
        self.target_angle = 0

        self.setXRange(-self.max_range, self.max_range)
        self.setYRange(-self.max_range, self.max_range)

        # Sweep animation timer
        self.sweep_timer = QTimer()
        self.sweep_timer.timeout.connect(self._update_sweep)
        self.sweep_timer.start(50)

    def setTarget(self, distance_cm):
        """Update sonar with detected target distance."""
        self.target_distance = min(distance_cm, self.max_range * 0.95)
        # Place target at current sweep angle
        tx = self.target_distance * np.cos(self.target_angle)
        ty = self.target_distance * np.sin(self.target_angle)
        self.target_dot.setData([tx], [ty])

    def _update_sweep(self):
        """Animate the sonar sweep."""
        self.sweep_angle += 3
        if self.sweep_angle >= 360:
            self.sweep_angle = 0

        self.target_angle = self.sweep_angle * np.pi / 180.0

        # Draw sweep line
        sx = [0, self.max_range * 0.95 * np.cos(self.target_angle)]
        sy = [0, self.max_range * 0.95 * np.sin(self.target_angle)]
        self.sweep_line.setData(sx, sy)

        # Update beam line (horizontal reference)
        bx = [0, self.max_range * 0.95]
        by = [0, 0]
        self.beam_line.setData(bx, by)

        # Move target to current sweep position if we have a distance
        if self.target_distance > 0:
            tx = self.target_distance * np.cos(self.target_angle)
            ty = self.target_distance * np.sin(self.target_angle)
            self.target_dot.setData([tx], [ty])


# ═══════════════════════════════════════════════════════════
#  Video Display Widget
# ═══════════════════════════════════════════════════════════
class VideoWidget(QLabel):
    """Displays video frames received via UDP."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(640, 480)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background-color: #0a0f19; border: 2px solid #1a3a5c; border-radius: 6px;")
        self.setText("NO VIDEO SIGNAL")
        self.setStyleSheet(
            "background-color: #0a0f19; border: 2px solid #1a3a5c; border-radius: 6px;"
            "color: #2a4a6a; font-size: 18px; font-weight: bold;"
        )
        self.current_frame = None
        self.recording = False
        self.video_writer = None
        self.record_filename = None

    def updateFrame(self, jpeg_data):
        """Decode and display a JPEG frame."""
        if not HAS_CV2:
            return

        try:
            nparr = np.frombuffer(jpeg_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None:
                return

            self.current_frame = frame

            # Write to video if recording
            if self.recording and self.video_writer:
                self.video_writer.write(frame)

            # Convert to QImage
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            q_img = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

            # Scale to fit widget while maintaining aspect ratio
            scaled = q_img.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.setPixmap(QPixmap.fromImage(scaled))

        except Exception:
            pass

    def startRecording(self, filename):
        """Start recording video to file."""
        if not HAS_CV2 or self.current_frame is None:
            return False

        try:
            h, w = self.current_frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(filename, fourcc, 20.0, (w, h))
            self.record_filename = filename
            self.recording = True
            return True
        except Exception:
            return False

    def startRecordingWithFrame(self, width=640, height=480, filename=None):
        """Start recording even without a current frame (uses default size)."""
        if not HAS_CV2:
            return False
        try:
            if filename is None:
                filename = os.path.join(VIDEO_DIR,
                                        f"MAGO_{datetime.now().strftime('%Y%m%d_%H%M%S')}.avi")
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(filename, fourcc, 20.0, (width, height))
            self.record_filename = filename
            self.recording = True
            return True
        except Exception:
            return False

    def stopRecording(self):
        """Stop recording video."""
        self.recording = False
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        fn = self.record_filename
        self.record_filename = None
        return fn

    def capturePhoto(self):
        """Capture current frame as a photo."""
        if self.current_frame is None:
            return None

        filename = os.path.join(PHOTO_DIR,
                                f"MAGO_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
        cv2.imwrite(filename, self.current_frame)
        return filename


# ═══════════════════════════════════════════════════════════
#  Connection Dialog
# ═══════════════════════════════════════════════════════════
class ConnectionDialog(QDialog):
    """Dialog for entering PC1 IP and pilot name."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("MAGO V1 - Connection Setup")
        self.setFixedSize(450, 280)
        self.setStyleSheet("""
            QDialog {
                background-color: #0d1b2a;
            }
            QLabel {
                color: #c0d8f0;
                font-size: 13px;
            }
            QLineEdit {
                background-color: #1b2838;
                color: #e0f0ff;
                border: 1px solid #2a4a6c;
                border-radius: 4px;
                padding: 6px;
                font-size: 13px;
            }
            QPushButton {
                background-color: #1a5276;
                color: #e0f0ff;
                border: 1px solid #2a7ab5;
                border-radius: 4px;
                padding: 8px 20px;
                font-size: 13px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2a7ab5;
            }
            QPushButton:pressed {
                background-color: #0d3460;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setSpacing(12)

        # Title
        title = QLabel("MAGO V1 - Deep Sea Exploration")
        title.setFont(QFont("Consolas", 16, QFont.Bold))
        title.setStyleSheet("color: #4fc3f7;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        subtitle = QLabel("Connection Configuration")
        subtitle.setStyleSheet("color: #7ab5d6; font-size: 12px;")
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)

        layout.addSpacing(10)

        # Form
        form = QFormLayout()
        form.setSpacing(10)

        self.ip_input = QLineEdit(DEFAULT_PC1_IP)
        self.ip_input.setPlaceholderText("e.g., 192.168.1.10")
        form.addRow("PC1 IP Address:", self.ip_input)

        self.pilot_input = QLineEdit("Pilot")
        self.pilot_input.setPlaceholderText("Enter pilot name")
        form.addRow("Pilot Name:", self.pilot_input)

        layout.addLayout(form)

        layout.addSpacing(10)

        # Buttons
        btn_layout = QHBoxLayout()
        connect_btn = QPushButton("CONNECT")
        connect_btn.clicked.connect(self.accept)
        cancel_btn = QPushButton("CANCEL")
        cancel_btn.clicked.connect(self.reject)
        btn_layout.addStretch()
        btn_layout.addWidget(connect_btn)
        btn_layout.addWidget(cancel_btn)
        layout.addLayout(btn_layout)


# ═══════════════════════════════════════════════════════════
#  Splash Screen (No QPropertyAnimation - simple timer)
# ═══════════════════════════════════════════════════════════
class MagoSplashScreen(QSplashScreen):
    """Simple splash screen without animations that crash weak GPUs."""

    def __init__(self):
        pixmap = QPixmap(600, 350)
        pixmap.fill(QColor(10, 20, 40))

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        # Gradient background
        grad = QLinearGradient(0, 0, 0, 350)
        grad.setColorAt(0, QColor(10, 25, 50))
        grad.setColorAt(0.5, QColor(15, 35, 65))
        grad.setColorAt(1, QColor(5, 15, 35))
        painter.fillRect(0, 0, 600, 350, grad)

        # Border
        painter.setPen(QPen(QColor(30, 100, 180), 2))
        painter.drawRect(1, 1, 598, 348)

        # Title
        painter.setFont(QFont("Consolas", 32, QFont.Bold))
        painter.setPen(QColor(80, 200, 255))
        painter.drawText(QRectF(0, 80, 600, 60), Qt.AlignCenter, "MAGO V1")

        # Subtitle
        painter.setFont(QFont("Consolas", 14))
        painter.setPen(QColor(120, 180, 220))
        painter.drawText(QRectF(0, 150, 600, 30), Qt.AlignCenter, "Deep Sea Exploration System")

        # Author
        painter.setFont(QFont("Consolas", 11))
        painter.setPen(QColor(80, 140, 180))
        painter.drawText(QRectF(0, 210, 600, 25), Qt.AlignCenter, "by Sohayb Shaaben")

        # Loading text
        painter.setFont(QFont("Consolas", 10))
        painter.setPen(QColor(60, 120, 170))
        painter.drawText(QRectF(0, 280, 600, 20), Qt.AlignCenter, "Initializing systems...")

        # Decorative line
        painter.setPen(QPen(QColor(40, 120, 200, 150), 1))
        painter.drawLine(150, 190, 450, 190)

        painter.end()

        super().__init__(pixmap)


# ═══════════════════════════════════════════════════════════
#  Main Window
# ═══════════════════════════════════════════════════════════
class MainWindow(QMainWindow):
    """Main control station window."""

    def __init__(self, pc1_ip, pilot_name):
        super().__init__()
        self.pc1_ip = pc1_ip
        self.pilot_name = pilot_name

        # State
        self.connected = False
        self.mission_active = False
        self.manual_recording = False
        self.current_movement = "IDLE"
        self.led_state = False
        self.thruster_power = 1500

        # Sensor data
        self.gyro_gx = 0.0
        self.gyro_gy = 0.0
        self.gyro_gz = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.sonar_distance = 0.0
        self.temperature = 0.0
        self.flow_rate = 0.0
        self.flow_volume = 0.0
        self.pressure_kpa = 0.0
        self.pressure_psi = 0.0
        self.depth = 0.0

        # Mission timer
        self.mission_timer = QElapsedTimer()
        self.mission_elapsed = 0

        # Network
        self.bridge = SignalBridge()
        self.network = NetworkClient(self.bridge)
        self.bridge.data_received.connect(self._on_data_received)
        self.bridge.frame_received.connect(self._on_frame_received)
        self.bridge.connection_changed.connect(self._on_connection_changed)
        self.bridge.status_message.connect(self._on_status_message)

        # Screen recording
        self.screen_rec_process = None

        self._init_ui()
        self._init_timers()

        # Connect to PC1 after a short delay
        QTimer.singleShot(500, lambda: self.network.connect(pc1_ip))

        # Key tracking
        self.pressed_keys = set()

    def _init_ui(self):
        """Build the complete GUI layout."""
        self.setWindowTitle("MAGO V1 - Deep Sea Exploration System")
        self.setFixedSize(WINDOW_WIDTH, WINDOW_HEIGHT)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0d1b2a;
            }
            QLabel {
                color: #c0d8f0;
                font-size: 12px;
            }
            QPushButton {
                background-color: #1a3a5c;
                color: #e0f0ff;
                border: 1px solid #2a5a8c;
                border-radius: 5px;
                padding: 8px 16px;
                font-size: 12px;
                font-weight: bold;
                min-height: 28px;
            }
            QPushButton:hover {
                background-color: #2a5a8c;
            }
            QPushButton:pressed {
                background-color: #0d2a4c;
            }
            QPushButton:checked {
                background-color: #c0392b;
                border-color: #e74c3c;
            }
            QSlider::groove:horizontal {
                height: 8px;
                background: #1b2838;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #2a7ab5;
                border: 1px solid #4a9ad5;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QSlider::sub-page:horizontal {
                background: #1a5276;
                border-radius: 4px;
            }
            QGroupBox {
                color: #7ab5d6;
                border: 1px solid #1a3a5c;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(4)
        main_layout.setContentsMargins(8, 4, 8, 4)

        # ──── Header ────
        self._build_header(main_layout)

        # ──── Main Content (Video + Right Panel) ────
        content_layout = QHBoxLayout()
        content_layout.setSpacing(6)

        # Left: Video (55%)
        self._build_video_panel(content_layout)

        # Right: Instruments + Data (45%)
        self._build_right_panel(content_layout)

        main_layout.addLayout(content_layout, stretch=1)

        # ──── Bottom Controls ────
        self._build_bottom_controls(main_layout)

        # ──── Footer ────
        self._build_footer(main_layout)

    def _build_header(self, parent_layout):
        """Build the header bar with status info."""
        header = QFrame()
        header.setStyleSheet(
            "QFrame { background-color: #0f2133; border: 1px solid #1a3a5c; border-radius: 4px; }"
        )
        header.setFixedHeight(48)
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(12, 2, 12, 2)

        # Connection status
        self.lbl_connection = QLabel("● DISCONNECTED")
        self.lbl_connection.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 13px;")
        h_layout.addWidget(self.lbl_connection)

        h_layout.addWidget(self._vseparator())

        # Pilot name
        lbl_pilot = QLabel(f"👤 Pilot: {self.pilot_name}")
        lbl_pilot.setStyleSheet("color: #7ab5d6; font-size: 13px; font-weight: bold;")
        h_layout.addWidget(lbl_pilot)

        h_layout.addWidget(self._vseparator())

        # System date/time
        self.lbl_datetime = QLabel()
        self.lbl_datetime.setStyleSheet("color: #8eb8d8; font-size: 13px;")
        h_layout.addWidget(self.lbl_datetime)

        h_layout.addStretch()

        # Mission time
        self.lbl_mission_time = QLabel("⏱ Mission: 00:00:00")
        self.lbl_mission_time.setStyleSheet(
            "color: #f39c12; font-size: 14px; font-weight: bold;"
        )
        h_layout.addWidget(self.lbl_mission_time)

        parent_layout.addWidget(header)

    def _build_video_panel(self, parent_layout):
        """Build the video display panel (left side)."""
        video_frame = QFrame()
        video_frame.setStyleSheet(
            "QFrame { background-color: #0a0f19; border: 2px solid #1a3a5c; border-radius: 6px; }"
        )
        v_layout = QVBoxLayout(video_frame)
        v_layout.setContentsMargins(2, 2, 2, 2)

        # Video label
        lbl_title = QLabel("📹 LIVE CAMERA FEED")
        lbl_title.setStyleSheet(
            "color: #4fc3f7; font-size: 12px; font-weight: bold; "
            "background: transparent; border: none;"
        )
        lbl_title.setAlignment(Qt.AlignCenter)
        v_layout.addWidget(lbl_title)

        self.video_widget = VideoWidget()
        v_layout.addWidget(self.video_widget, stretch=1)

        # Recording indicator
        self.lbl_rec_indicator = QLabel("")
        self.lbl_rec_indicator.setStyleSheet(
            "color: #e74c3c; font-size: 11px; font-weight: bold; background: transparent; border: none;"
        )
        self.lbl_rec_indicator.setAlignment(Qt.AlignCenter)
        v_layout.addWidget(self.lbl_rec_indicator)

        parent_layout.addWidget(video_frame, stretch=55)

    def _build_right_panel(self, parent_layout):
        """Build the right instrument and data panel."""
        right_frame = QFrame()
        right_frame.setStyleSheet(
            "QFrame { background-color: #0f1923; border: 1px solid #1a3a5c; border-radius: 6px; }"
        )
        r_layout = QVBoxLayout(right_frame)
        r_layout.setSpacing(4)
        r_layout.setContentsMargins(6, 6, 6, 6)

        # ── Gyro Compass ──
        gyro_group = QGroupBox("GYRO COMPASS")
        gyro_group.setStyleSheet(
            "QGroupBox { color: #4fc3f7; border: 1px solid #1a3a5c; border-radius: 5px; "
            "margin-top: 8px; padding-top: 14px; font-weight: bold; font-size: 11px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
        )
        gyro_layout = QVBoxLayout(gyro_group)
        gyro_layout.setSpacing(2)

        self.compass_widget = CompassWidget()
        self.compass_widget.setFixedHeight(200)
        gyro_layout.addWidget(self.compass_widget)

        self.lbl_orientation = QLabel("Roll: 0.0°  Pitch: 0.0°  Yaw: 0.0°")
        self.lbl_orientation.setStyleSheet(
            "color: #8eb8d8; font-size: 11px; font-weight: bold; background: transparent;"
        )
        self.lbl_orientation.setAlignment(Qt.AlignCenter)
        gyro_layout.addWidget(self.lbl_orientation)

        r_layout.addWidget(gyro_group)

        # ── Sonar ──
        sonar_group = QGroupBox("SONAR")
        sonar_group.setStyleSheet(
            "QGroupBox { color: #4fc3f7; border: 1px solid #1a3a5c; border-radius: 5px; "
            "margin-top: 8px; padding-top: 14px; font-weight: bold; font-size: 11px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
        )
        sonar_layout = QVBoxLayout(sonar_group)
        sonar_layout.setSpacing(2)

        self.sonar_widget = SonarWidget()
        self.sonar_widget.setFixedHeight(180)
        sonar_layout.addWidget(self.sonar_widget)

        r_layout.addWidget(sonar_group)

        # ── Data Panel ──
        data_group = QGroupBox("SENSOR DATA")
        data_group.setStyleSheet(
            "QGroupBox { color: #4fc3f7; border: 1px solid #1a3a5c; border-radius: 5px; "
            "margin-top: 8px; padding-top: 14px; font-weight: bold; font-size: 11px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
        )
        data_grid = QGridLayout(data_group)
        data_grid.setSpacing(4)

        self.data_labels = {}
        sensors = [
            ("IMU Gyro X", "0.00 deg/s"),
            ("IMU Gyro Y", "0.00 deg/s"),
            ("IMU Gyro Z", "0.00 deg/s"),
            ("Roll", "0.00°"),
            ("Pitch", "0.00°"),
            ("Yaw", "0.00°"),
            ("Temperature", "0.00 °C"),
            ("Sonar Dist", "0.00 cm"),
            ("Flow Rate", "0.00 L/min"),
            ("Flow Volume", "0.00 L"),
            ("Pressure", "0.00 kPa"),
            ("Depth", "0.00 m"),
        ]

        for i, (name, default) in enumerate(sensors):
            row = i // 2
            col = (i % 2) * 2

            lbl_name = QLabel(f"{name}:")
            lbl_name.setStyleSheet(
                "color: #5a8aa8; font-size: 10px; font-weight: bold; background: transparent;"
            )
            data_grid.addWidget(lbl_name, row, col)

            lbl_val = QLabel(default)
            lbl_val.setStyleSheet(
                "color: #4fc3f7; font-size: 10px; font-weight: bold; background: transparent;"
            )
            lbl_val.setAlignment(Qt.AlignRight)
            data_grid.addWidget(lbl_val, row, col + 1)
            self.data_labels[name] = lbl_val

        r_layout.addWidget(data_group)

        # ── Movement Indicator ──
        move_frame = QFrame()
        move_frame.setStyleSheet(
            "QFrame { background-color: #111b27; border: 1px solid #1a3a5c; border-radius: 4px; }"
        )
        move_layout = QHBoxLayout(move_frame)
        move_layout.setContentsMargins(8, 4, 8, 4)

        move_label_title = QLabel("MOVEMENT:")
        move_label_title.setStyleSheet(
            "color: #5a8aa8; font-size: 11px; font-weight: bold; background: transparent;"
        )
        move_layout.addWidget(move_label_title)

        self.lbl_movement = QLabel("IDLE")
        self.lbl_movement.setStyleSheet(
            "color: #2ecc71; font-size: 14px; font-weight: bold; background: transparent;"
        )
        self.lbl_movement.setAlignment(Qt.AlignCenter)
        move_layout.addWidget(self.lbl_movement, stretch=1)

        r_layout.addWidget(move_frame)

        parent_layout.addWidget(right_frame, stretch=45)

    def _build_bottom_controls(self, parent_layout):
        """Build the bottom control bar."""
        bottom = QFrame()
        bottom.setStyleSheet(
            "QFrame { background-color: #0f2133; border: 1px solid #1a3a5c; border-radius: 4px; }"
        )
        bottom.setFixedHeight(65)
        b_layout = QHBoxLayout(bottom)
        b_layout.setContentsMargins(10, 4, 10, 4)
        b_layout.setSpacing(8)

        # Mission Start/Stop
        self.btn_mission = QPushButton(">> MISSION START")
        self.btn_mission.setFixedWidth(180)
        self.btn_mission.setStyleSheet(
            "QPushButton { background-color: #1a6b3c; color: #e0f0ff; border: 1px solid #2a9b5c; "
            "border-radius: 5px; padding: 8px 16px; font-size: 12px; font-weight: bold; }"
            "QPushButton:hover { background-color: #2a9b5c; }"
            "QPushButton:checked { background-color: #c0392b; border-color: #e74c3c; }"
        )
        self.btn_mission.setCheckable(True)
        self.btn_mission.clicked.connect(self._on_mission_toggle)
        b_layout.addWidget(self.btn_mission)

        # Manual Record
        self.btn_rec = QPushButton("REC MANUAL")
        self.btn_rec.setFixedWidth(130)
        self.btn_rec.setStyleSheet(
            "QPushButton { background-color: #7b241c; color: #e0f0ff; border: 1px solid #c0392b; "
            "border-radius: 5px; padding: 8px 12px; font-size: 12px; font-weight: bold; }"
            "QPushButton:hover { background-color: #c0392b; }"
        )
        self.btn_rec.clicked.connect(self._on_rec_toggle)
        b_layout.addWidget(self.btn_rec)

        # Take Photo
        self.btn_photo = QPushButton("CAM TAKE PHOTO")
        self.btn_photo.setFixedWidth(150)
        self.btn_photo.clicked.connect(self._on_take_photo)
        b_layout.addWidget(self.btn_photo)

        # LED On
        self.btn_led_on = QPushButton("LED ON")
        self.btn_led_on.setFixedWidth(90)
        self.btn_led_on.setStyleSheet(
            "QPushButton { background-color: #1a3a5c; color: #e0f0ff; border: 1px solid #2a5a8c; "
            "border-radius: 5px; padding: 8px 12px; font-size: 12px; font-weight: bold; }"
            "QPushButton:hover { background-color: #2a5a8c; }"
        )
        self.btn_led_on.clicked.connect(lambda: self._send_led(True))
        b_layout.addWidget(self.btn_led_on)

        # LED Off
        self.btn_led_off = QPushButton("LED OFF")
        self.btn_led_off.setFixedWidth(90)
        self.btn_led_off.setStyleSheet(
            "QPushButton { background-color: #1a3a5c; color: #e0f0ff; border: 1px solid #2a5a8c; "
            "border-radius: 5px; padding: 8px 12px; font-size: 12px; font-weight: bold; }"
            "QPushButton:hover { background-color: #2a5a8c; }"
        )
        self.btn_led_off.clicked.connect(lambda: self._send_led(False))
        b_layout.addWidget(self.btn_led_off)

        b_layout.addStretch()

        # Thruster Power Slider
        lbl_thruster = QLabel("THRUSTER POWER:")
        lbl_thruster.setStyleSheet("color: #7ab5d6; font-size: 11px; font-weight: bold; background: transparent;")
        b_layout.addWidget(lbl_thruster)

        self.slider_thruster = QSlider(Qt.Horizontal)
        self.slider_thruster.setRange(1100, 1900)
        self.slider_thruster.setValue(1500)
        self.slider_thruster.setFixedWidth(200)
        self.slider_thruster.valueChanged.connect(self._on_thruster_slider)
        b_layout.addWidget(self.slider_thruster)

        self.lbl_thruster_val = QLabel("1500 µs")
        self.lbl_thruster_val.setStyleSheet(
            "color: #4fc3f7; font-size: 11px; font-weight: bold; background: transparent;"
        )
        self.lbl_thruster_val.setFixedWidth(60)
        b_layout.addWidget(self.lbl_thruster_val)

        parent_layout.addWidget(bottom)

    def _build_footer(self, parent_layout):
        """Build the footer bar."""
        footer = QFrame()
        footer.setStyleSheet("QFrame { background: transparent; }")
        footer.setFixedHeight(20)
        f_layout = QHBoxLayout(footer)
        f_layout.setContentsMargins(0, 0, 10, 0)

        f_layout.addStretch()

        lbl_footer = QLabel("Created by Sohayb Shaaben")
        lbl_footer.setStyleSheet("color: #3a5a7a; font-size: 10px; background: transparent;")
        lbl_footer.setAlignment(Qt.AlignRight)
        f_layout.addWidget(lbl_footer)

        parent_layout.addWidget(footer)

    @staticmethod
    def _vseparator():
        """Create a vertical separator widget."""
        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet("color: #1a3a5c; background: transparent;")
        sep.setFixedWidth(2)
        return sep

    # ─────────────── Timers ───────────────
    def _init_timers(self):
        """Initialize periodic update timers."""
        # Clock update
        self.clock_timer = QTimer()
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(1000)
        self._update_clock()

        # Mission time update
        self.mission_display_timer = QTimer()
        self.mission_display_timer.timeout.connect(self._update_mission_time)

    # ─────────────── Clock ───────────────
    def _update_clock(self):
        """Update system date/time display."""
        now = datetime.now()
        self.lbl_datetime.setText(now.strftime("📅 %Y-%m-%d  🕐 %H:%M:%S"))

    # ─────────────── Mission Time ───────────────
    def _update_mission_time(self):
        """Update mission elapsed time display."""
        if self.mission_active:
            self.mission_elapsed = self.mission_timer.elapsed()
        secs = self.mission_elapsed // 1000
        h = secs // 3600
        m = (secs % 3600) // 60
        s = secs % 60
        self.lbl_mission_time.setText(f"⏱ Mission: {h:02d}:{m:02d}:{s:02d}")

    # ─────────────── Data Handling ───────────────
    def _on_data_received(self, line):
        """Parse incoming sensor data from PC1."""
        try:
            if line.startswith("[GYRO]"):
                # [GYRO] Gx: <float> deg/s  Gy: <float> deg/s  Gz: <float> deg/s
                match = re.search(
                    r'Gx:\s*([-\d.]+)\s*deg/s\s*Gy:\s*([-\d.]+)\s*deg/s\s*Gz:\s*([-\d.]+)\s*deg/s',
                    line
                )
                if match:
                    self.gyro_gx = float(match.group(1))
                    self.gyro_gy = float(match.group(2))
                    self.gyro_gz = float(match.group(3))
                    self.data_labels["IMU Gyro X"].setText(f"{self.gyro_gx:.2f} deg/s")
                    self.data_labels["IMU Gyro Y"].setText(f"{self.gyro_gy:.2f} deg/s")
                    self.data_labels["IMU Gyro Z"].setText(f"{self.gyro_gz:.2f} deg/s")

            elif line.startswith("[ORIENT]"):
                # [ORIENT] Roll: <float> Pitch: <float> Yaw: <float> deg
                match = re.search(
                    r'Roll:\s*([-\d.]+)\s*Pitch:\s*([-\d.]+)\s*Yaw:\s*([-\d.]+)',
                    line
                )
                if match:
                    self.roll = float(match.group(1))
                    self.pitch = float(match.group(2))
                    self.yaw = float(match.group(3))
                    self.data_labels["Roll"].setText(f"{self.roll:.2f}°")
                    self.data_labels["Pitch"].setText(f"{self.pitch:.2f}°")
                    self.data_labels["Yaw"].setText(f"{self.yaw:.2f}°")
                    self.compass_widget.setOrientation(self.yaw, self.roll, self.pitch)
                    self.lbl_orientation.setText(
                        f"Roll: {self.roll:.1f}°  Pitch: {self.pitch:.1f}°  Yaw: {self.yaw:.1f}°"
                    )

            elif line.startswith("[SONAR]"):
                # [SONAR] Distance: <float> cm
                match = re.search(r'Distance:\s*([-\d.]+)\s*cm', line)
                if match:
                    self.sonar_distance = float(match.group(1))
                    self.data_labels["Sonar Dist"].setText(f"{self.sonar_distance:.2f} cm")
                    self.sonar_widget.setTarget(self.sonar_distance)

            elif line.startswith("[LM35]"):
                # [LM35] Temperature: <float> C
                match = re.search(r'Temperature:\s*([-\d.]+)\s*C', line)
                if match:
                    self.temperature = float(match.group(1))
                    self.data_labels["Temperature"].setText(f"{self.temperature:.2f} °C")

            elif line.startswith("[FLOW]"):
                # [FLOW] Flow: <float> L/min Vol: <float> L | Pres: <float> kPa (<float> psi) | Depth: <float> m
                match = re.search(
                    r'Flow:\s*([-\d.]+)\s*L/min\s*Vol:\s*([-\d.]+)\s*L\s*\|\s*'
                    r'Pres:\s*([-\d.]+)\s*kPa\s*\(([-\d.]+)\s*psi\)\s*\|\s*'
                    r'Depth:\s*([-\d.]+)\s*m',
                    line
                )
                if match:
                    self.flow_rate = float(match.group(1))
                    self.flow_volume = float(match.group(2))
                    self.pressure_kpa = float(match.group(3))
                    self.pressure_psi = float(match.group(4))
                    self.depth = float(match.group(5))
                    self.data_labels["Flow Rate"].setText(f"{self.flow_rate:.2f} L/min")
                    self.data_labels["Flow Volume"].setText(f"{self.flow_volume:.2f} L")
                    self.data_labels["Pressure"].setText(f"{self.pressure_kpa:.2f} kPa")
                    self.data_labels["Depth"].setText(f"{self.depth:.2f} m")

        except Exception:
            pass

    def _on_frame_received(self, jpeg_data):
        """Handle received video frame."""
        self.video_widget.updateFrame(jpeg_data)

    def _on_connection_changed(self, connected):
        """Handle connection state change."""
        self.connected = connected
        if connected:
            self.lbl_connection.setText("● CONNECTED")
            self.lbl_connection.setStyleSheet(
                "color: #2ecc71; font-weight: bold; font-size: 13px;"
            )
        else:
            self.lbl_connection.setText("● DISCONNECTED")
            self.lbl_connection.setStyleSheet(
                "color: #e74c3c; font-weight: bold; font-size: 13px;"
            )

    def _on_status_message(self, msg):
        """Handle status messages from network."""
        print(f"[STATUS] {msg}", flush=True)

    # ─────────────── Controls ───────────────
    def _on_mission_toggle(self, checked):
        """Toggle mission start/stop."""
        if checked:
            # Mission START
            self.mission_active = True
            self.mission_timer.start()
            self.mission_elapsed = 0
            self.mission_display_timer.start(100)
            self.btn_mission.setText("■ MISSION STOP")
            self.btn_mission.setStyleSheet(
                "QPushButton { background-color: #c0392b; color: #e0f0ff; "
                "border: 1px solid #e74c3c; border-radius: 5px; padding: 8px 16px; "
                "font-size: 12px; font-weight: bold; }"
                "QPushButton:hover { background-color: #e74c3c; }"
            )

            # Auto-start screen recording
            self._start_screen_recording()

        else:
            # Mission STOP
            self.mission_active = False
            self.mission_elapsed = self.mission_timer.elapsed()
            self.mission_timer.invalidate()
            self.mission_display_timer.stop()
            self._update_mission_time()
            self.btn_mission.setText(">> MISSION START")
            self.btn_mission.setStyleSheet(
                "QPushButton { background-color: #1a6b3c; color: #e0f0ff; "
                "border: 1px solid #2a9b5c; border-radius: 5px; padding: 8px 16px; "
                "font-size: 12px; font-weight: bold; }"
                "QPushButton:hover { background-color: #2a9b5c; }"
            )

            # Auto-stop screen recording
            self._stop_screen_recording()

    def _start_screen_recording(self):
        """Start screen recording using ffmpeg (if available)."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(VIDEO_DIR, f"Screen_Mission_{timestamp}.mp4")

        # Try ffmpeg for screen capture
        try:
            if sys.platform == "win32":
                cmd = [
                    "ffmpeg", "-y", "-f", "gdigrab", "-framerate", "20",
                    "-i", "desktop", "-c:v", "libx264", "-preset", "ultrafast",
                    "-crf", "28", filename
                ]
            else:
                cmd = [
                    "ffmpeg", "-y", "-f", "x11grab", "-framerate", "20",
                    "-video_size", f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}",
                    "-i", ":0.0", "-c:v", "libx264", "-preset", "ultrafast",
                    "-crf", "28", filename
                ]
            self.screen_rec_process = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            self.lbl_rec_indicator.setText("● RECORDING (Mission)")
            self.lbl_rec_indicator.setStyleSheet(
                "color: #e74c3c; font-size: 11px; font-weight: bold; background: transparent;"
            )
        except FileNotFoundError:
            # ffmpeg not available, try video widget recording
            self.video_widget.startRecordingWithFrame(filename=filename.replace('.mp4', '.avi'))
            self.lbl_rec_indicator.setText("● REC (Camera Only)")
            self.lbl_rec_indicator.setStyleSheet(
                "color: #e74c3c; font-size: 11px; font-weight: bold; background: transparent;"
            )
        except Exception:
            self.lbl_rec_indicator.setText("● REC FAILED")
            self.lbl_rec_indicator.setStyleSheet(
                "color: #f39c12; font-size: 11px; font-weight: bold; background: transparent;"
            )

    def _stop_screen_recording(self):
        """Stop screen recording."""
        if self.screen_rec_process:
            try:
                self.screen_rec_process.terminate()
                self.screen_rec_process.wait(timeout=5)
            except Exception:
                try:
                    self.screen_rec_process.kill()
                except Exception:
                    pass
            self.screen_rec_process = None

        if self.video_widget.recording:
            self.video_widget.stopRecording()

        self.lbl_rec_indicator.setText("")
        self.lbl_rec_indicator.setStyleSheet(
            "color: transparent; font-size: 11px; background: transparent;"
        )

    def _on_rec_toggle(self):
        """Toggle manual recording."""
        if not self.manual_recording:
            # Start manual recording
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(VIDEO_DIR, f"Manual_{timestamp}.avi")
            if self.video_widget.startRecordingWithFrame(filename=filename):
                self.manual_recording = True
                self.btn_rec.setText("■ STOP REC")
                self.btn_rec.setStyleSheet(
                    "QPushButton { background-color: #c0392b; color: #e0f0ff; "
                    "border: 1px solid #e74c3c; border-radius: 5px; padding: 8px 12px; "
                    "font-size: 12px; font-weight: bold; }"
                    "QPushButton:hover { background-color: #e74c3c; }"
                )
                self.lbl_rec_indicator.setText("● MANUAL REC")
                self.lbl_rec_indicator.setStyleSheet(
                    "color: #e74c3c; font-size: 11px; font-weight: bold; background: transparent;"
                )
        else:
            # Stop manual recording
            fn = self.video_widget.stopRecording()
            self.manual_recording = False
            self.btn_rec.setText("REC MANUAL")
            self.btn_rec.setStyleSheet(
                "QPushButton { background-color: #7b241c; color: #e0f0ff; "
                "border: 1px solid #c0392b; border-radius: 5px; padding: 8px 12px; "
                "font-size: 12px; font-weight: bold; }"
                "QPushButton:hover { background-color: #c0392b; }"
            )
            if not self.mission_active:
                self.lbl_rec_indicator.setText("")

    def _on_take_photo(self):
        """Capture a photo from the current video frame."""
        fn = self.video_widget.capturePhoto()
        if fn:
            self.lbl_rec_indicator.setText(f"📸 Photo saved!")
            self.lbl_rec_indicator.setStyleSheet(
                "color: #2ecc71; font-size: 11px; font-weight: bold; background: transparent;"
            )
            QTimer.singleShot(3000, lambda: self.lbl_rec_indicator.setText("") if not self.manual_recording and not self.mission_active else None)

    def _send_led(self, on):
        """Send LED on/off command."""
        self.led_state = on
        cmd = "led on" if on else "led off"
        self.network.send_command(cmd)
        if on:
            self.btn_led_on.setStyleSheet(
                "QPushButton { background-color: #2a9b5c; color: #e0f0ff; "
                "border: 1px solid #4acc7c; border-radius: 5px; padding: 8px 12px; "
                "font-size: 12px; font-weight: bold; }"
            )
            self.btn_led_off.setStyleSheet(
                "QPushButton { background-color: #1a3a5c; color: #e0f0ff; "
                "border: 1px solid #2a5a8c; border-radius: 5px; padding: 8px 12px; "
                "font-size: 12px; font-weight: bold; }"
            )
        else:
            self.btn_led_off.setStyleSheet(
                "QPushButton { background-color: #c0392b; color: #e0f0ff; "
                "border: 1px solid #e74c3c; border-radius: 5px; padding: 8px 12px; "
                "font-size: 12px; font-weight: bold; }"
            )
            self.btn_led_on.setStyleSheet(
                "QPushButton { background-color: #1a3a5c; color: #e0f0ff; "
                "border: 1px solid #2a5a8c; border-radius: 5px; padding: 8px 12px; "
                "font-size: 12px; font-weight: bold; }"
            )

    def _on_thruster_slider(self, value):
        """Handle thruster power slider change."""
        self.thruster_power = value
        self.lbl_thruster_val.setText(f"{value} µs")
        self.network.send_command(f"pwm:{value}")

    # ─────────────── Movement Commands ───────────────
    _MOVEMENT_MAP = {
        Qt.Key_Z: ('z', "FORWARD"),
        Qt.Key_S: ('s', "BACKWARD"),
        Qt.Key_D: ('d', "RIGHT"),
        Qt.Key_Q: ('q', "LEFT"),
        Qt.Key_E: ('e', "ROTATE RIGHT"),
        Qt.Key_A: ('a', "ROTATE LEFT"),
        Qt.Key_Space: (' ', "IDLE"),
    }

    _MOVEMENT_COLORS = {
        "FORWARD": "#2ecc71",
        "BACKWARD": "#e74c3c",
        "RIGHT": "#f39c12",
        "LEFT": "#3498db",
        "ROTATE RIGHT": "#9b59b6",
        "ROTATE LEFT": "#1abc9c",
        "IDLE": "#2ecc71",
    }

    def keyPressEvent(self, event):
        """Handle key press for movement commands."""
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in self._MOVEMENT_MAP and key not in self.pressed_keys:
            self.pressed_keys.add(key)
            cmd, state = self._MOVEMENT_MAP[key]
            self.current_movement = state
            self.lbl_movement.setText(state)
            color = self._MOVEMENT_COLORS.get(state, "#2ecc71")
            self.lbl_movement.setStyleSheet(
                f"color: {color}; font-size: 14px; font-weight: bold; background: transparent;"
            )
            self.network.send_command(cmd)
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """Handle key release - stop when movement key released."""
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in self._MOVEMENT_MAP:
            self.pressed_keys.discard(key)
            # If no other movement keys pressed, stop
            if not self.pressed_keys:
                self.current_movement = "IDLE"
                self.lbl_movement.setText("IDLE")
                self.lbl_movement.setStyleSheet(
                    "color: #2ecc71; font-size: 14px; font-weight: bold; background: transparent;"
                )
                self.network.send_command(' ')
        super().keyReleaseEvent(event)

    # ─────────────── Cleanup ───────────────
    def closeEvent(self, event):
        """Clean up on window close."""
        self.network.disconnect()
        if self.mission_active:
            self._stop_screen_recording()
        if self.manual_recording:
            self.video_widget.stopRecording()
        event.accept()


# ═══════════════════════════════════════════════════════════
#  Application Entry Point
# ═══════════════════════════════════════════════════════════
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # ── Splash Screen (simple, no animation) ──
    splash = MagoSplashScreen()
    splash.show()
    app.processEvents()

    # CRITICAL: Simple timer close, no QPropertyAnimation/QGraphicsOpacityEffect
    # (These crash on weak graphics cards like Intel Celeron N4100)
    splash_close_timer = [None]  # Use list to avoid nonlocal in Python 3

    def close_splash():
        splash.close()

    QTimer.singleShot(3000, close_splash)

    # ── Connection Dialog ──
    dialog = ConnectionDialog()
    if dialog.exec_() != QDialog.Accepted:
        sys.exit(0)

    pc1_ip = dialog.ip_input.text().strip()
    pilot_name = dialog.pilot_input.text().strip() or "Pilot"

    if not pc1_ip:
        pc1_ip = DEFAULT_PC1_IP

    # ── Main Window ──
    window = MainWindow(pc1_ip, pilot_name)
    window.show()

    # Close splash immediately when main window shows
    splash.close()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
