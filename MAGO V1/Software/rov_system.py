

import sys
import os
import socket
import threading
import serial
import cv2
import time
import math
import re
import numpy as np
import mss
import pyqtgraph.opengl as gl
import pyqtgraph as pg
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QMessageBox, QDialog, 
                             QInputDialog, QGroupBox, QGridLayout, QFrame, 
                             QLineEdit, QSlider, QGraphicsOpacityEffect) # ADDED QGraphicsOpacityEffect
from PyQt5.QtCore import (Qt, pyqtSignal, QObject, QTimer, QElapsedTimer, 
                          QPropertyAnimation) # ADDED QPropertyAnimation
from PyQt5.QtGui import QImage, QPixmap, QFont
from datetime import datetime

# ==========================================
# CONFIGURATION
# ==========================================
PC1_IP = "0.0.0.0"
PC2_IP = "192.168.1.2"   # <--- CHANGE THIS TO PC1's ACTUAL IP ADDRESS
TCP_PORT = 5005
UDP_PORT = 5006
SERIAL_PORT = "COM6"
BAUD_RATE = 115200

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)

# Automatically get the name of the computer (e.g., "DESKTOP-ABC123" or "LAPTOP-ROV")
PC_NAME = socket.gethostname()

# Create specific folders for Videos and Photos named after the computer
VIDEO_DIR = os.path.join(PROJECT_DIR, "Data", "Videos", PC_NAME)
PHOTO_DIR = os.path.join(PROJECT_DIR, "Data", "Photos", PC_NAME)

forwardSpeed = 1600
invertedSpeed = 1400

# ==========================================
# NETWORK SIGNAL WORKERS
# ==========================================
class CommunicationSignals(QObject):
    data_received = pyqtSignal(str)
    video_received = pyqtSignal(QPixmap)

signals = CommunicationSignals()

# ==========================================
# 1. SPLASH SCREEN ANIMATION (MAGO V1)
# ==========================================
# ==========================================
# 1. SPLASH SCREEN ANIMATION (MAGO V1)
# ==========================================
class SplashAnimation(QWidget):
    finished = pyqtSignal() # Signal to tell the app we are done

    def __init__(self):
        super().__init__()
        self.setFixedSize(800, 400)
        self.setStyleSheet("background-color: #0b0c10;")
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        
        # Center on screen
        screen_geometry = QApplication.desktop().screenGeometry()
        x = (screen_geometry.width() - self.width()) // 2
        y = (screen_geometry.height() - self.height()) // 2
        self.move(x, y)

        layout = QVBoxLayout(self)
        
        # --- Container for both texts ---
        text_container = QWidget()
        text_container.setStyleSheet("background-color: transparent;")
        text_layout = QVBoxLayout(text_container)
        
        # Main Title
        self.title_label = QLabel("MAGO V1")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("color: #64ffda; font-size: 90px; font-weight: bold; font-family: Consolas;")
        
        # Subtitle (Small Text)
        self.subtitle_label = QLabel("Deep Sea Exploration System") # <--- CHANGE THIS TEXT TO WHATEVER YOU WANT
        self.subtitle_label.setAlignment(Qt.AlignCenter)
        self.subtitle_label.setStyleSheet("color: #45a29e; font-size: 24px; font-weight: normal; font-family: Consolas;")
         
         # Subtitle (Small Text)
        self.subtitle_labe = QLabel("by Sohayb Shaben") # <--- CHANGE THIS TEXT TO WHATEVER YOU WANT
        self.subtitle_labe.setAlignment(Qt.AlignCenter)
        self.subtitle_labe.setStyleSheet("color: #45a29e; font-size: 24px; font-weight: normal; font-family: Consolas;")

        text_layout.addWidget(self.title_label)
        text_layout.addWidget(self.subtitle_label)
        text_layout.addWidget(self.subtitle_labe)
        text_layout.setContentsMargins(0, 0, 0, 0)
        
        # Opacity effect for fade animation (Applied to the container so both texts fade together)
        self.effect = QGraphicsOpacityEffect()
        text_container.setGraphicsEffect(self.effect)
        
        layout.addWidget(text_container)
        
        # Animation setup (Fade in -> Hold -> Fade out)
        self.animation = QPropertyAnimation(self.effect, b"opacity")
        self.animation.setDuration(3000) # 3 seconds total
        self.animation.setStartValue(0.0)
        self.animation.setKeyValueAt(0.3, 1.0) # 30% time: fully visible
        self.animation.setKeyValueAt(0.7, 1.0) # 70% time: still visible
        self.animation.setEndValue(0.0)         # 100% time: fully invisible
        
        self.animation.finished.connect(self.close)
        self.animation.finished.connect(self.finished.emit) # Emit signal when done

    def start_animation(self):
        self.show()
        self.animation.start()
# ==========================================
# BOOT & PILOT DIALOG
# ==========================================
class StartupDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MAGO V1 - System Boot")
        self.setFixedSize(450, 250)
        self.setStyleSheet("background-color: #121212; color: #00bcd4; font-family: Consolas;")
        self.selected_mode = "PC1"
        self.pilot_name = "Unknown Pilot"

        layout = QVBoxLayout()

        name_layout = QHBoxLayout()
        name_label = QLabel("Pilot Name:")
        name_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.name_text = QLineEdit("Pilot_1")
        self.name_text.setStyleSheet("background-color: #2d2d2d; color: #64ffda; padding: 5px; border: 1px solid #00bcd4;")
        name_layout.addWidget(name_label)
        name_layout.addWidget(self.name_text)
        layout.addLayout(name_layout)

        self.label = QLabel("Booting PC1 - ROV Base in 15 seconds...\nSelect mode manually to override:")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 14px; margin-top: 10px;")
        layout.addWidget(self.label)

        self.btn_pc1 = QPushButton("PC1 - ROV Base (Arduino + Cam)")
        self.btn_pc1.setStyleSheet(self.btn_style("#1e1e1e"))
        self.btn_pc1.clicked.connect(self.select_pc1)

        self.btn_pc2 = QPushButton("PC2 - Control Station (GUI)")
        self.btn_pc2.setStyleSheet(self.btn_style("#1e1e1e"))
        self.btn_pc2.clicked.connect(self.select_pc2)

        layout.addWidget(self.btn_pc1)
        layout.addWidget(self.btn_pc2)
        self.setLayout(layout)

        self.timer = QTimer()
        self.countdown = 15 
        self.timer.timeout.connect(self.update_countdown)
        self.timer.start(1000)

    def btn_style(self, bg):
        return f"QPushButton {{ background-color: {bg}; color: #64ffda; font-weight: bold; padding: 10px; border: 1px solid #00bcd4; }} QPushButton:hover {{ background-color: #00bcd4; color: black; }}"

    def update_countdown(self):
        self.countdown -= 1
        if self.countdown <= 0:
            self.timer.stop()
            self.accept()
        else:
            self.label.setText(f"Booting PC1 - ROV Base in {self.countdown} seconds...\nSelect mode manually to override:")

    def select_pc1(self):
        self.pilot_name = self.name_text.text()
        self.selected_mode = "PC1"
        self.timer.stop()
        self.accept()

    def select_pc2(self):
        self.pilot_name = self.name_text.text()
        self.selected_mode = "PC2"
        self.timer.stop()
        self.accept()

# ==========================================
# PC 1: ROV BASE STATION LOGIC
# ==========================================
def run_pc1():
    print("--- STARTING PC1: ROV BASE ---")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Arduino on {SERIAL_PORT}")
    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        return

    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_sock.bind((PC1_IP, TCP_PORT))
    tcp_sock.listen(1)
    print(f"TCP Server listening on {TCP_PORT}...")

    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind((PC1_IP, UDP_PORT))

    print("Waiting for PC2 to connect...")
    conn, addr = tcp_sock.accept()
    print(f"PC2 Connected from {addr}")

    def serial_to_tcp():
        while True:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        conn.sendall((line + "\n").encode('utf-8'))
            except: break

    def tcp_to_serial():
        buffer = ""
        while True:
            try:
                data = conn.recv(1024).decode('utf-8')
                if not data: break
                buffer += data
                while '\n' in buffer:
                    cmd, buffer = buffer.split('\n', 1)
                    ser.write((cmd.strip() + '\n').encode('utf-8'))
            except: break

    def video_streamer():
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if not ret: continue
            frame = cv2.resize(frame, (640, 480))
            _, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            data = encoded.tobytes()
            if len(data) < 65000:
                udp_sock.sendto(data, (addr[0], UDP_PORT + 1))

    threading.Thread(target=serial_to_tcp, daemon=True).start()
    threading.Thread(target=tcp_to_serial, daemon=True).start()
    threading.Thread(target=video_streamer, daemon=True).start()
    input("Press Enter to exit PC1...\n")

# ==========================================
# SCREEN RECORDER THREAD
# ==========================================
class ScreenRecorder(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.is_recording = False
        self.writer = None
        self.sct = mss.mss()
        
    def start_recording(self, geometry):
        os.makedirs(VIDEO_DIR, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(VIDEO_DIR, f"MAGO_Mission_{timestamp}.avi")
        
        w = geometry.width() if geometry.width() % 2 == 0 else geometry.width() - 1
        h = geometry.height() if geometry.height() % 2 == 0 else geometry.height() - 1
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.writer = cv2.VideoWriter(filepath, fourcc, 20.0, (w, h))
        self.geometry_dict = {
            "top": geometry.top(), "left": geometry.left(),
            "width": geometry.width(), "height": geometry.height()
        }
        self.is_recording = True
        self.start() 

    def stop_recording(self):
        self.is_recording = False
        time.sleep(0.2)
        if self.writer:
            self.writer.release()
            self.writer = None

    def run(self):
        while self.is_recording:
            try:
                img = np.array(self.sct.grab(self.geometry_dict))
                img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                if self.writer: self.writer.write(img_bgr)
                time.sleep(1/20) 
            except Exception as e:
                break

# ==========================================
# PC 2: CONTROL STATION GUI
# ==========================================
class ROVControlGUI(QWidget):
    def __init__(self, pilot_name):
        super().__init__()
        self.setWindowTitle("MAGO V1 - ROV Control Station")
        self.setFixedSize(1920, 1000) 
        
        self.pilot_name = pilot_name
        self.is_mission_active = False

        self.setStyleSheet("""
            QWidget { background-color: #0f1218; color: #c5c6c7; font-family: Segoe UI; }
            QLabel { font-size: 13px; }
            QGroupBox { 
                border: 1px solid #1f2833; 
                border-radius: 4px; 
                margin-top: 10px; 
                padding-top: 15px; 
                font-weight: bold; 
                color: #64ffda;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton { 
                background-color: #1a1d23; color: #c5c6c7; border: 1px solid #45a29e; 
                border-radius: 4px; padding: 8px; font-weight: bold; min-width: 80px;
            }
            QPushButton:hover { background-color: #45a29e; color: #0b0c10; }
            QPushButton:pressed { background-color: #64ffda; }
            QSlider::groove:horizontal {
                border: 1px solid #45a29e;
                height: 8px;
                background: #1a1d23;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #64ffda;
                border: 1px solid #0f1218;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QSlider::sub-page:horizontal {
                background: #45a29e;
                border-radius: 4px;
            }
        """)
        
        self.active_key = None
        self.tcp_sock = None
        self.recorder = ScreenRecorder()
        
        self.gyro_roll = 0
        self.gyro_pitch = 0
        self.gyro_yaw = 0
        self.last_gyro_time = time.time()

        self.mission_timer = QElapsedTimer()
        self.timer_update = QTimer()
        self.timer_update.timeout.connect(self.update_times)
        self.timer_update.start(1000)

        self.init_ui()
        self.connect_to_pc1()

    def init_ui(self):
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(10, 5, 10, 5) 
        main_layout.setSpacing(5)

        # --- Header Bar ---
        header_frame = QFrame()
        header_frame.setStyleSheet("background-color: #1a1d23; border-bottom: 2px solid #45a29e; border-radius: 2px;")
        header_layout = QHBoxLayout(header_frame)
        header_layout.setContentsMargins(10, 2, 10, 2)

        self.status_label = QLabel("● OFFLINE")
        self.status_label.setStyleSheet("color: #ff4c4c; font-weight: bold; font-size: 14px;")
        
        self.pilot_label = QLabel(f"Pilot: {self.pilot_name}")
        self.pilot_label.setStyleSheet("color: #64ffda; font-weight: bold; font-size: 14px;")

        self.sys_time_label = QLabel("SYS: --")
        self.sys_time_label.setStyleSheet("color: #ffffff; font-weight: bold; font-size: 14px; font-family: Consolas;")

        self.mission_time_label = QLabel("MISSION: 00:00:00")
        self.mission_time_label.setStyleSheet("color: #64ffda; font-weight: bold; font-size: 14px; font-family: Consolas;")

        header_layout.addWidget(self.status_label)
        header_layout.addStretch()
        header_layout.addWidget(self.pilot_label)
        header_layout.addStretch()
        header_layout.addWidget(self.sys_time_label)
        header_layout.addWidget(self.mission_time_label)
        main_layout.addWidget(header_frame)

        # --- Middle Content ---
        middle_layout = QHBoxLayout()
        middle_layout.setSpacing(10)

        # LEFT: Camera Feed
        cam_group = QGroupBox("LIVE CAMERA FEED")
        cam_layout = QVBoxLayout(cam_group)
        self.video_label = QLabel("INITIALIZING CAM...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: #000000; border: 1px solid #1f2833;")
        cam_layout.addWidget(self.video_label)
        middle_layout.addWidget(cam_group, 55) 

        # RIGHT: Data & Visualizations
        right_layout = QVBoxLayout()
        right_layout.setSpacing(10)

        # 3D Gyroscope
        gyro_group = QGroupBox("3D GYROSCOPE ORIENTATION (Drag to Rotate)")
        gyro_layout = QVBoxLayout(gyro_group)
        self.gyro_view = gl.GLViewWidget()
        self.gyro_view.setMinimumHeight(340)
        self.gyro_view.setBackgroundColor('#0f1218')
        try:
            self.init_3d_gyro() 
        except Exception as e:
            print(f"3D Gyro Init Error: {e}")
        gyro_layout.addWidget(self.gyro_view)
        right_layout.addWidget(gyro_group, 50)

        # 2D Sonar
        sonar_group = QGroupBox("2D SONAR RADAR (Scroll to Zoom)")
        sonar_layout = QVBoxLayout(sonar_group)
        self.sonar_plot = pg.PlotWidget()
        self.sonar_plot.setMinimumHeight(250)
        self.sonar_plot.setBackground('#0f1218')
        self.sonar_plot.setAspectLocked()
        try:
            self.init_2d_sonar() 
        except Exception as e:
            print(f"2D Sonar Init Error: {e}")
        sonar_layout.addWidget(self.sonar_plot)
        right_layout.addWidget(sonar_group, 35)

        # Sensor & Movement Panel 
        data_group = QGroupBox("SYSTEM DATA & CONTROLS")
        data_layout = QGridLayout(data_group)
        data_layout.setSpacing(8)
        
        self.imu_label = QLabel("IMU: 0.0 | 0.0 | 0.0 deg/s")
        self.temp_label = QLabel("TEMP: 0.0 C")
        self.dist_label = QLabel("SONAR: 0.0 cm")
        self.flow_label = QLabel("FLOW: 0.00 L/min")
        self.pres_label = QLabel("PRES: 0.00 kPa (0.00 psi)")
        self.depth_label = QLabel("DEPTH: 0.00 m")
        
        self.movement_label = QLabel("MOVEMENT: IDLE")
        self.movement_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #64ffda;")

        for lbl in [self.imu_label, self.temp_label, self.dist_label, 
                    self.flow_label, self.pres_label, self.depth_label]:
            lbl.setStyleSheet("background-color: #1f2833; padding: 8px; border-radius: 3px; font-weight: bold;")

        data_layout.addWidget(self.imu_label, 0, 0, 1, 2)
        data_layout.addWidget(self.temp_label, 0, 2, 1, 2)
        data_layout.addWidget(self.dist_label, 0, 4, 1, 2)
        data_layout.addWidget(self.flow_label, 1, 0, 1, 2)
        data_layout.addWidget(self.pres_label, 1, 2, 1, 2)
        data_layout.addWidget(self.depth_label, 1, 4, 1, 2)
        data_layout.addWidget(self.movement_label, 2, 0, 1, 6)
        
        right_layout.addWidget(data_group, 15)

        middle_layout.addLayout(right_layout, 45) 
        main_layout.addLayout(middle_layout, 85) 

        # --- Bottom Controls & Thruster Slider ---
        bottom_frame = QFrame()
        bottom_frame.setStyleSheet("background-color: #1a1d23; border-top: 2px solid #45a29e; border-radius: 2px;")
        bottom_layout = QHBoxLayout(bottom_frame)
        bottom_layout.setContentsMargins(15, 5, 15, 5)
        bottom_layout.setSpacing(20)

        self.btn_mission = QPushButton(">> MISSION START")
        self.btn_mission.setStyleSheet("QPushButton { background-color: #1a1d23; color: #64ffda; border: 2px solid #64ffda; padding: 10px; font-size: 16px; } QPushButton:hover { background-color: #64ffda; color: black; }")
        self.btn_mission.clicked.connect(self.toggle_mission)

        self.btn_record = QPushButton("REC MANUAL")
        self.btn_record.setStyleSheet("QPushButton { border-color: #ff4c4c; color: #ff4c4c; } QPushButton:hover { background-color: #ff4c4c; color: black; }")
        self.btn_record.clicked.connect(self.toggle_recording)

        self.btn_photo = QPushButton("CAM TAKE PHOTO")
        self.btn_photo.setStyleSheet("QPushButton { border-color: #64ffda; color: #64ffda; } QPushButton:hover { background-color: #64ffda; color: black; }")
        self.btn_photo.clicked.connect(self.take_photo)

        self.btn_led_on = QPushButton("LED ON")
        self.btn_led_on.clicked.connect(lambda: self.send_command("led on"))

        self.btn_led_off = QPushButton("LED OFF")
        self.btn_led_off.clicked.connect(lambda: self.send_command("led off"))

        slider_label = QLabel("THRUSTER POWER:")
        slider_label.setStyleSheet("color: #64ffda; font-weight: bold; font-size: 14px;")
        
        self.thruster_slider = QSlider(Qt.Horizontal)
        self.thruster_slider.setRange(1100, 1900)
        self.thruster_slider.setValue(1600)
        self.thruster_slider.setMinimumWidth(300)
        self.thruster_slider.valueChanged.connect(self.update_thruster_speed)
        
        self.slider_value_label = QLabel("1600 us")
        self.slider_value_label.setStyleSheet("color: #ffffff; font-weight: bold; font-size: 14px; font-family: Consolas;")

        bottom_layout.addWidget(self.btn_mission)
        bottom_layout.addWidget(self.btn_record)
        bottom_layout.addWidget(self.btn_photo)
        bottom_layout.addStretch()
        bottom_layout.addWidget(self.btn_led_on)
        bottom_layout.addWidget(self.btn_led_off)
        bottom_layout.addStretch()
        bottom_layout.addWidget(slider_label)
        bottom_layout.addWidget(self.thruster_slider)
        bottom_layout.addWidget(self.slider_value_label)

        main_layout.addWidget(bottom_frame, 15) 

        # --- Credit Footer ---
        credit_label = QLabel("Created by Sohayb Shaaben")
        credit_label.setAlignment(Qt.AlignRight)
        credit_label.setStyleSheet("color: #3b8585; font-size: 12px; padding-right: 15px; font-style: italic;")
        main_layout.addWidget(credit_label)

        self.setLayout(main_layout)

    # --- 3D & 2D INITIALIZATIONS ---
    def init_3d_gyro(self):
        grid = gl.GLGridItem()
        grid.setSize(15, 15, 1)
        grid.setSpacing(1, 1, 1)
        grid.setColor((69, 162, 158, 50))
        self.gyro_view.addItem(grid)
        
        axis_x = gl.GLLinePlotItem(pos=np.array([[0,0,0],[4,0,0]], dtype=np.float32), color=(1,0,0,1), width=2, antialias=True)
        axis_y = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,4,0]], dtype=np.float32), color=(0,1,0,1), width=2, antialias=True)
        axis_z = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,0,4]], dtype=np.float32), color=(0,0,1,1), width=2, antialias=True)
        self.gyro_view.addItem(axis_x)
        self.gyro_view.addItem(axis_y)
        self.gyro_view.addItem(axis_z)

        verts = []
        faces = []
        colors = []

        n_sides = 16
        r = 0.8
        length = 3.0
        verts.append([-length/2, 0, 0]) 
        verts.append([length/2, 0, 0])  
        
        cap_back_start = 2
        cap_front_start = 2 + n_sides
        
        for i in range(n_sides):
            angle = 2 * math.pi * i / n_sides
            y = r * math.cos(angle)
            z = r * math.sin(angle)
            verts.append([-length/2, y, z]) 
            verts.append([length/2, y, z])  
            
            i_back_cur = cap_back_start + i
            i_front_cur = cap_front_start + i
            i_back_next = cap_back_start + (i + 1) % n_sides
            i_front_next = cap_front_start + (i + 1) % n_sides
            
            faces.append([i_back_cur, i_back_next, i_front_next])
            faces.append([i_back_cur, i_front_next, i_front_cur])
            colors.append([1.0, 0.5, 0.0, 0.8]) 
            colors.append([1.0, 0.5, 0.0, 0.8]) 
            
            faces.append([0, i_back_next, i_back_cur])
            colors.append([1.0, 1.0, 0.0, 0.8]) 
            
            faces.append([1, i_front_cur, i_front_next])
            colors.append([1.0, 0.0, 0.0, 0.8]) 

        wing_chord = 1.5
        wing_span = 2.5
        wing_y_start = r
        wing_thickness = 0.05

        rw_v1 = len(verts); verts.append([wing_chord/2, wing_y_start, wing_thickness])
        rw_v2 = len(verts); verts.append([wing_chord/2, wing_y_start + wing_span, wing_thickness])
        rw_v3 = len(verts); verts.append([-wing_chord/2, wing_y_start + wing_span, wing_thickness])
        rw_v4 = len(verts); verts.append([-wing_chord/2, wing_y_start, wing_thickness])
        faces.append([rw_v1, rw_v2, rw_v3]); colors.append([0.2, 0.8, 0.2, 1.0]) 
        faces.append([rw_v1, rw_v3, rw_v4]); colors.append([0.2, 0.8, 0.2, 1.0])
        
        rw_v5 = len(verts); verts.append([wing_chord/2, wing_y_start, -wing_thickness])
        rw_v6 = len(verts); verts.append([wing_chord/2, wing_y_start + wing_span, -wing_thickness])
        rw_v7 = len(verts); verts.append([-wing_chord/2, wing_y_start + wing_span, -wing_thickness])
        rw_v8 = len(verts); verts.append([-wing_chord/2, wing_y_start, -wing_thickness])
        faces.append([rw_v5, rw_v7, rw_v6]); colors.append([0.1, 0.6, 0.1, 1.0]) 
        faces.append([rw_v5, rw_v8, rw_v7]); colors.append([0.1, 0.6, 0.1, 1.0])

        lw_v1 = len(verts); verts.append([wing_chord/2, -wing_y_start, wing_thickness])
        lw_v2 = len(verts); verts.append([wing_chord/2, -(wing_y_start + wing_span), wing_thickness])
        lw_v3 = len(verts); verts.append([-wing_chord/2, -(wing_y_start + wing_span), wing_thickness])
        lw_v4 = len(verts); verts.append([-wing_chord/2, -wing_y_start, wing_thickness])
        faces.append([lw_v1, lw_v3, lw_v2]); colors.append([0.2, 0.8, 0.2, 1.0])
        faces.append([lw_v1, lw_v4, lw_v3]); colors.append([0.2, 0.8, 0.2, 1.0])
        
        lw_v5 = len(verts); verts.append([wing_chord/2, -wing_y_start, -wing_thickness])
        lw_v6 = len(verts); verts.append([wing_chord/2, -(wing_y_start + wing_span), -wing_thickness])
        lw_v7 = len(verts); verts.append([-wing_chord/2, -(wing_y_start + wing_span), -wing_thickness])
        lw_v8 = len(verts); verts.append([-wing_chord/2, -wing_y_start, -wing_thickness])
        faces.append([lw_v5, lw_v6, lw_v7]); colors.append([0.1, 0.6, 0.1, 1.0])
        faces.append([lw_v5, lw_v7, lw_v8]); colors.append([0.1, 0.6, 0.1, 1.0])

        verts_np = np.array(verts, dtype=np.float32)
        faces_np = np.array(faces, dtype=np.uint32)
        colors_np = np.array(colors, dtype=np.float32)

        self.gyro_mesh = gl.GLMeshItem(vertexes=verts_np, faces=faces_np, faceColors=colors_np, smooth=False, drawEdges=True, edgeColor=(0.1, 0.1, 0.1, 1))
        self.gyro_view.addItem(self.gyro_mesh)
        self.gyro_view.setCameraPosition(distance=12, elevation=30, azimuth=45)

    def init_2d_sonar(self):
        self.sonar_plot.setXRange(-10, 10)
        self.sonar_plot.setYRange(-10, 10)
        self.sonar_plot.setLabel('bottom', 'X (m)')
        self.sonar_plot.setLabel('left', 'Y (m)')

        for r in [2, 4, 6, 8]:
            theta = np.linspace(0, 2 * np.pi, 100)
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            circle = pg.PlotCurveItem(x, y, pen=pg.mkPen(69, 162, 158, 50))
            self.sonar_plot.addItem(circle)

        rov_marker = pg.ScatterPlotItem()
        rov_marker.setData([0], [0], size=10, brush=pg.mkBrush(0, 188, 212))
        self.sonar_plot.addItem(rov_marker)

        self.sonar_target = pg.ScatterPlotItem()
        self.sonar_target.setData([0], [5], size=15, brush=pg.mkBrush(255, 0, 0))
        self.sonar_plot.addItem(self.sonar_target)

        self.sonar_beam = pg.PlotCurveItem()
        self.sonar_beam.setData([0, 0], [0, 5], pen=pg.mkPen(0, 188, 212, 150))
        self.sonar_plot.addItem(self.sonar_beam)

    # --- MISSION, TIME & SLIDER LOGIC ---
    def update_times(self):
        now = datetime.now()
        sys_str = now.strftime("%Y-%m-%d %H:%M:%S")
        self.sys_time_label.setText(f"SYS: {sys_str}")

        if self.is_mission_active:
            elapsed_ms = self.mission_timer.elapsed()
            s = int((elapsed_ms / 1000) % 60)
            m = int((elapsed_ms / 1000 / 60) % 60)
            h = int(elapsed_ms / 1000 / 3600)
            self.mission_time_label.setText(f"MISSION: {h:02d}:{m:02d}:{s:02d}")
        else:
            self.mission_time_label.setText("MISSION: 00:00:00")

    def toggle_mission(self):
        if not self.is_mission_active:
            self.is_mission_active = True
            self.mission_timer.restart() 
            self.btn_mission.setText("|| MISSION STOP")
            self.btn_mission.setStyleSheet("QPushButton { background-color: #ff4c4c; color: white; border: 2px solid #ff4c4c; padding: 10px; font-size: 16px; } QPushButton:hover { background-color: #ff0000; color: black; }")
            
            if not self.recorder.is_recording:
                self.toggle_recording()
        else:
            self.is_mission_active = False
            self.btn_mission.setText(">> MISSION START")
            self.btn_mission.setStyleSheet("QPushButton { background-color: #1a1d23; color: #64ffda; border: 2px solid #64ffda; padding: 10px; font-size: 16px; } QPushButton:hover { background-color: #64ffda; color: black; }")
            
            if self.recorder.is_recording:
                self.toggle_recording()

    def update_thruster_speed(self, val):
        global forwardSpeed, invertedSpeed
        forwardSpeed = val
        invertedSpeed = 3000 - val
        self.slider_value_label.setText(f"{val} us")

    # --- RECORDING & PHOTO ---
    def toggle_recording(self):
        if self.recorder.is_recording:
            self.recorder.stop_recording()
            self.btn_record.setText("REC MANUAL")
            self.status_label.setText("● SAVED")
            self.status_label.setStyleSheet("color: #64ffda; font-weight: bold; font-size: 14px;")
        else:
            self.recorder.start_recording(self.geometry())
            self.btn_record.setText("STOP REC")
            self.status_label.setText("● RECORDING")
            self.status_label.setStyleSheet("color: #ff4c4c; font-weight: bold; font-size: 14px;")

    def take_photo(self):
        os.makedirs(PHOTO_DIR, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(PHOTO_DIR, f"MAGO_Photo_{timestamp}.png")
        pixmap = self.grab()
        pixmap.save(filepath)
        self.status_label.setText("● PHOTO SAVED")
        self.status_label.setStyleSheet("color: #64ffda; font-weight: bold; font-size: 14px;")

    # --- NETWORKING ---
    def connect_to_pc1(self):
        try:
            self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_sock.connect((PC2_IP, TCP_PORT))
            self.status_label.setText("● ONLINE")
            self.status_label.setStyleSheet("color: #64ffda; font-weight: bold; font-size: 14px;")
            threading.Thread(target=self.receive_tcp_data, daemon=True).start()
            threading.Thread(target=self.receive_udp_video, daemon=True).start()
        except Exception as e:
            self.status_label.setText(f"● CONNECTION FAILED")
            self.status_label.setStyleSheet("color: #ff4c4c; font-weight: bold; font-size: 14px;")

    def receive_tcp_data(self):
        buffer = ""
        while True:
            try:
                data = self.tcp_sock.recv(1024).decode('utf-8')
                if not data: break
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    signals.data_received.emit(line.strip())
            except: break

    def receive_udp_video(self):
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.bind(("0.0.0.0", UDP_PORT + 1))
        while True:
            data, _ = udp_sock.recvfrom(65536)
            if data:
                nparr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if frame is not None:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = frame.shape
                    qimg = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
                    pixmap = QPixmap.fromImage(qimg).scaled(self.video_label.width(), self.video_label.height(), Qt.KeepAspectRatio)
                    signals.video_received.emit(pixmap)

    def send_command(self, cmd):
        if self.tcp_sock:
            try: self.tcp_sock.sendall((cmd + "\n").encode('utf-8'))
            except: pass

    # --- GUI & 3D DATA UPDATES ---
    def update_sensor_gui(line):
        try:
            current_time = time.time()
            if "[GYRO]" in line:
                data = line.split(']')[1].strip()
                parts = data.split("  ")
                gx = float(parts[0].split(":")[1].replace("deg/s", "").strip())
                gy = float(parts[1].split(":")[1].replace("deg/s", "").strip())
                gz = float(parts[2].split(":")[1].replace("deg/s", "").strip())
                
                dt = current_time - gui.last_gyro_time
                gui.last_gyro_time = current_time
                
                gui.gyro_roll  += gx * dt
                gui.gyro_pitch += gy * dt
                gui.gyro_yaw   += gz * dt
                
                gui.gyro_mesh.resetTransform()
                gui.gyro_mesh.rotate(gui.gyro_roll, 1, 0, 0)  
                gui.gyro_mesh.rotate(gui.gyro_pitch, 0, 1, 0) 
                gui.gyro_mesh.rotate(gui.gyro_yaw, 0, 0, 1)   
                
                gui.imu_label.setText(f"IMU: R:{gui.gyro_roll:.1f} P:{gui.gyro_pitch:.1f} Y:{gui.gyro_yaw:.1f} deg")

            elif "[SONAR]" in line:
                data = line.split(']')[1].strip()
                if "Distance" in data:
                    dist_cm = float(data.replace("Distance:", "").replace("cm", "").strip())
                    dist_m = dist_cm / 100.0 
                    
                    gui.sonar_target.setData([0], [dist_m])
                    gui.sonar_beam.setData([0, 0], [0, dist_m])
                    gui.dist_label.setText(f"SONAR: {dist_cm:.1f} cm")

            elif "[LM35]" in line:
                data = line.split(']')[1].strip()
                gui.temp_label.setText(f"TEMP: {data}")

            elif "[FLOW]" in line:
                data = line.split(']')[1].strip()
                try:
                    flow_match = re.search(r'Flow: ([\d\.]+) L/min', data)
                    pres_match = re.search(r'Pres: ([\d\.]+) kPa \(([\d\.]+) psi\)', data)
                    depth_match = re.search(r'Depth: ([\d\.]+) m', data)
                    
                    if flow_match:
                        flow_val = flow_match.group(1)
                        gui.flow_label.setText(f"FLOW: {flow_val} L/min")
                        
                    if pres_match:
                        kpa_val = pres_match.group(1)
                        psi_val = pres_match.group(2)
                        gui.pres_label.setText(f"PRES: {kpa_val} kPa ({psi_val} psi)")
                        
                    if depth_match:
                        depth_val = depth_match.group(1)
                        gui.depth_label.setText(f"DEPTH: {depth_val} m")
                except Exception as e:
                    print(f"Regex Parse Error: {e}")

            elif "[LED]" in line or "[THR]" in line:
                pass 
        except: pass

    def update_video_gui(pixmap):
        gui.video_label.setPixmap(pixmap)

    signals.data_received.connect(update_sensor_gui)
    signals.video_received.connect(update_video_gui)

    # --- KEYBOARD INPUT ---
    def keyPressEvent(self, event):
        key = event.text().lower()
        if key in ['z', 's', 'd', 'q', 'e', 'a', ' ']:
            if self.active_key != key:
                self.active_key = key
                self.send_command(key)
                directions = {'z': 'FORWARD', 's': 'BACKWARD', 'd': 'RIGHT', 'q': 'LEFT', 'e': 'STRAFE R', 'a': 'STRAFE L', ' ': 'IDLE'}
                self.movement_label.setText(f"MOVEMENT: {directions.get(key, 'IDLE')}")

    def keyReleaseEvent(self, event):
        key = event.text().lower()
        if key == self.active_key:
            self.active_key = None
            self.send_command(' ')
            self.movement_label.setText("MOVEMENT: IDLE")

# ==========================================
# MAIN EXECUTION (Event Driven - Safer!)
# ==========================================
def start_main_process():
    dialog = StartupDialog()
    if dialog.exec_() == QDialog.Accepted:
        if dialog.selected_mode == "PC1":
            run_pc1()
        else:
            global gui
            gui = ROVControlGUI(dialog.pilot_name)
            gui.show()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Play Animation, then when finished, trigger the main boot dialog
    splash = SplashAnimation()
    splash.finished.connect(start_main_process)
    splash.start_animation()
    
    sys.exit(app.exec_())