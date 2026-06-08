#!/usr/bin/env python3
"""
============================================================
 MAGO V1 - PC1 Base Station (LattePanda Delta 3)
 Deep Sea Exploration System
 by Sohayb Shaaben
============================================================

 Headless script running on the ROV's LattePanda.
 - Reads Arduino sensor data via Serial and forwards via TCP
 - Captures webcam via iVCam and streams via UDP
 - Receives TCP commands from PC2 and forwards to Arduino

 Communication:
   TCP Port 5005: Bidirectional text data
   UDP Port 5006: Video stream (PC1 sends)
   UDP Port 5007: Video stream (PC2 receives)
"""

import sys
import os
import socket
import threading
import time
import struct
import traceback

# ─────────────── Configuration ───────────────
ARDUINO_SERIAL_PORT = "/dev/ttyUSB0"  # Adjust for your system (ttyACM0, COM3, etc.)
ARDUINO_BAUD = 115200
TCP_PORT = 5005
UDP_SEND_PORT = 5006      # PC1 sends on this port
UDP_RECV_PORT = 5007      # PC2 receives on this port
PC2_IP = "192.168.1.100"  # Default PC2 IP; will be updated when PC2 connects
VIDEO_FPS = 20
VIDEO_JPEG_QUALITY = 70
VIDEO_MAX_SIZE = 65000     # Max UDP packet size for JPEG frames

# ─────────────── Globals ───────────────
running = True
serial_conn = None
tcp_client_socket = None
tcp_server_socket = None
udp_video_socket = None
pc2_address = None  # (IP, port) of connected PC2

lock_serial = threading.Lock()
lock_tcp_client = threading.Lock()
lock_pc2_addr = threading.Lock()


def log(msg):
    """Thread-safe logging to stdout."""
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ═══════════════════════════════════════════════════════════
#  Serial (Arduino) Thread
# ═══════════════════════════════════════════════════════════
def serial_reader_thread():
    """Read lines from Arduino and forward to PC2 via TCP."""
    global serial_conn, running
    import serial

    while running:
        try:
            if serial_conn is None or not serial_conn.is_open:
                log("Serial: Attempting to connect to Arduino...")
                try:
                    serial_conn = serial.Serial(
                        ARDUINO_SERIAL_PORT,
                        ARDUINO_BAUD,
                        timeout=1.0
                    )
                    time.sleep(2)  # Wait for Arduino reset
                    log(f"Serial: Connected to {ARDUINO_SERIAL_PORT}")
                except Exception as e:
                    log(f"Serial: Connection failed - {e}")
                    time.sleep(3)
                    continue

            line = serial_conn.readline()
            if line:
                try:
                    decoded = line.decode('utf-8', errors='replace').strip()
                except Exception:
                    decoded = line.decode('latin-1', errors='replace').strip()

                if decoded:
                    log(f"Serial RX: {decoded}")
                    # Forward to PC2 via TCP
                    send_to_pc2((decoded + "\n").encode('utf-8'))

        except Exception as e:
            log(f"Serial: Error - {e}")
            if serial_conn and serial_conn.is_open:
                try:
                    serial_conn.close()
                except Exception:
                    pass
            serial_conn = None
            time.sleep(2)


def send_to_arduino(data_str):
    """Send a command string to Arduino via Serial."""
    global serial_conn
    with lock_serial:
        if serial_conn and serial_conn.is_open:
            try:
                serial_conn.write((data_str + "\n").encode('utf-8'))
                log(f"Serial TX: {data_str}")
            except Exception as e:
                log(f"Serial: Send error - {e}")


# ═══════════════════════════════════════════════════════════
#  TCP Server Thread (bidirectional with PC2)
# ═══════════════════════════════════════════════════════════
def tcp_server_thread():
    """Listen for PC2 TCP connections, handle bidirectional data."""
    global tcp_server_socket, tcp_client_socket, running, pc2_address

    tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_server_socket.bind(("0.0.0.0", TCP_PORT))
    tcp_server_socket.listen(1)
    tcp_server_socket.settimeout(1.0)
    log(f"TCP Server: Listening on port {TCP_PORT}")

    while running:
        # Accept connections
        try:
            client_sock, client_addr = tcp_server_socket.accept()
            log(f"TCP Server: PC2 connected from {client_addr}")
            with lock_tcp_client:
                if tcp_client_socket:
                    try:
                        tcp_client_socket.close()
                    except Exception:
                        pass
                tcp_client_socket = client_sock
                tcp_client_socket.settimeout(0.5)

            with lock_pc2_addr:
                pc2_address = client_addr

            # Handle incoming data from PC2
            _handle_pc2_commands(client_sock)

        except socket.timeout:
            continue
        except Exception as e:
            if running:
                log(f"TCP Server: Accept error - {e}")
            time.sleep(0.5)


def _handle_pc2_commands(client_sock):
    """Read commands from PC2 and forward to Arduino."""
    global running
    buffer = ""

    while running:
        try:
            data = client_sock.recv(4096)
            if not data:
                log("TCP: PC2 disconnected")
                break

            buffer += data.decode('utf-8', errors='replace')

            # Process complete lines
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()
                if line:
                    log(f"TCP RX from PC2: {line}")
                    # Forward to Arduino
                    send_to_arduino(line)

        except socket.timeout:
            continue
        except ConnectionResetError:
            log("TCP: PC2 connection reset")
            break
        except Exception as e:
            if running:
                log(f"TCP: Command read error - {e}")
            break

    try:
        client_sock.close()
    except Exception:
        pass

    with lock_tcp_client:
        global tcp_client_socket
        if tcp_client_socket == client_sock:
            tcp_client_socket = None


def send_to_pc2(data_bytes):
    """Send data to PC2 via TCP."""
    with lock_tcp_client:
        if tcp_client_socket:
            try:
                tcp_client_socket.sendall(data_bytes)
            except Exception as e:
                log(f"TCP: Send to PC2 error - {e}")


# ═══════════════════════════════════════════════════════════
#  Video Stream Thread (UDP)
# ═══════════════════════════════════════════════════════════
def video_stream_thread():
    """Capture webcam frames and stream via UDP to PC2."""
    global running, pc2_address, udp_video_socket
    import cv2
    import numpy as np

    # CRITICAL: iVCam crash fix - use CAP_MSMF backend and test indices
    cap = None
    working_index = -1

    for idx in [0, 1, 2]:
        try:
            log(f"Video: Trying camera index {idx} with CAP_MSMF...")
            test_cap = cv2.VideoCapture(idx, cv2.CAP_MSMF)
            if test_cap.isOpened():
                ret, frame = test_cap.read()
                if ret and frame is not None:
                    log(f"Video: Camera index {idx} works!")
                    cap = test_cap
                    working_index = idx
                    break
                else:
                    test_cap.release()
            else:
                test_cap.release()
        except Exception as e:
            log(f"Video: Camera index {idx} failed - {e}")
            continue

    if cap is None:
        log("Video: FATAL - No working camera found!")
        log("Video: Trying default backend as last resort...")
        try:
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                log("Video: No camera available at all. Video disabled.")
                return
        except Exception as e:
            log(f"Video: Last resort failed - {e}")
            return

    # CRITICAL: Set buffer size to 1 to reduce latency
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        log("Video: Could not set buffer size (non-critical)")

    # Set camera resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # UDP Socket for video
    udp_video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_video_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536 * 4)

    # Encode parameters for JPEG
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, VIDEO_JPEG_QUALITY]

    frame_interval = 1.0 / VIDEO_FPS
    last_frame_time = time.time()

    log("Video: Streaming started")

    while running:
        try:
            # Target FPS control
            elapsed = time.time() - last_frame_time
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)
            last_frame_time = time.time()

            ret, frame = cap.read()
            if not ret or frame is None:
                log("Video: Frame read failed, retrying...")
                time.sleep(0.1)
                continue

            # Encode to JPEG
            _, jpeg_data = cv2.imencode('.jpg', frame, encode_params)
            frame_bytes = jpeg_data.tobytes()

            # Check size constraint
            if len(frame_bytes) > VIDEO_MAX_SIZE:
                # Reduce quality and re-encode
                encode_params[1] = max(20, VIDEO_JPEG_QUALITY - 20)
                _, jpeg_data = cv2.imencode('.jpg', frame, encode_params)
                frame_bytes = jpeg_data.tobytes()
                encode_params[1] = VIDEO_JPEG_QUALITY  # Reset

            # Get PC2 address
            target_ip = None
            with lock_pc2_addr:
                if pc2_address:
                    target_ip = pc2_address[0]

            if target_ip is None:
                # No PC2 connected yet, skip sending
                continue

            # Send frame via UDP
            # Header: 4 bytes frame size (for reassembly if needed), then JPEG data
            # Simple approach: send as single datagram (under 65KB)
            try:
                udp_video_socket.sendto(frame_bytes, (target_ip, UDP_RECV_PORT))
            except Exception as e:
                log(f"Video: UDP send error - {e}")

        except Exception as e:
            if running:
                log(f"Video: Error - {e}")
                traceback.print_exc()
            time.sleep(0.1)

    # Cleanup
    if cap:
        cap.release()
    if udp_video_socket:
        udp_video_socket.close()
    log("Video: Streaming stopped")


# ═══════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════
def main():
    global running

    log("=" * 60)
    log("  MAGO V1 - PC1 Base Station")
    log("  Deep Sea Exploration System")
    log("  by Sohayb Shaaben")
    log("=" * 60)

    # Detect serial port
    global ARDUINO_SERIAL_PORT
    if not os.path.exists(ARDUINO_SERIAL_PORT):
        # Try common alternatives
        for port in ["/dev/ttyACM0", "/dev/ttyUSB1", "/dev/ttyS0", "COM3", "COM4"]:
            if os.path.exists(port):
                ARDUINO_SERIAL_PORT = port
                log(f"Auto-detected serial port: {port}")
                break
        else:
            log(f"Warning: Default port {ARDUINO_SERIAL_PORT} not found.")
            log("Will keep trying... (edit ARDUINO_SERIAL_PORT in script)")

    # Start threads
    t_serial = threading.Thread(target=serial_reader_thread, daemon=True)
    t_tcp = threading.Thread(target=tcp_server_thread, daemon=True)
    t_video = threading.Thread(target=video_stream_thread, daemon=True)

    t_serial.start()
    t_tcp.start()
    time.sleep(1)  # Give TCP server time to start before video
    t_video.start()

    log("All threads started. Press Ctrl+C to exit.")

    try:
        while running:
            time.sleep(1)
    except KeyboardInterrupt:
        log("Shutting down...")
        running = False
        time.sleep(1)

    # Cleanup
    global serial_conn, tcp_server_socket, tcp_client_socket, udp_video_socket
    if serial_conn and serial_conn.is_open:
        serial_conn.close()
    if tcp_client_socket:
        try:
            tcp_client_socket.close()
        except Exception:
            pass
    if tcp_server_socket:
        try:
            tcp_server_socket.close()
        except Exception:
            pass
    if udp_video_socket:
        try:
            udp_video_socket.close()
        except Exception:
            pass

    log("PC1 Base Station stopped.")


if __name__ == "__main__":
    main()
