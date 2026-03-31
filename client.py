#!/usr/bin/env python3
"""USAR Client - runs on your laptop.
Combines keyboard teleoperation with optional video streaming.
Pressing 'c' activates CV mode: arrow detection runs on the laptop,
then high-level TURN/DRIVE commands are sent to the Pi.
"""
import socket
import subprocess
import os
import threading
import numpy as np
import cv2
import time
import json

# ----------------------
# CONFIGURATION - UPDATE THESE
# ----------------------
RASPI_IP = "192.168.137.178"  # <-- change to your robot's IP
HELLO_PORT = 7123
TELEOP_PORT = 7124
STATUS_PORT = 7125

WIDTH = 640
HEIGHT = 480

STREAM_TYPE = "ps3"  # "ps3" or "logitech"
ENABLE_VIDEO = True  # set True once camera is set up

# Speed levels (power values 1-5)
SPEED_LEVELS = {
    ord('1'): 1.0,
    ord('2'): 2.0,
    ord('3'): 3.0,
    ord('4'): 4.0,
    ord('5'): 5.0,
}

current_power = 3.0  # default speed

# ----------------------
# NETWORK SETUP
# ----------------------
teleop_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_cmd(cmd: str, **kwargs):
    msg = {"cmd": cmd, "power": current_power}
    msg.update(kwargs)
    teleop_sock.sendto(json.dumps(msg).encode(), (RASPI_IP, TELEOP_PORT))

# Status listener (receives TURN_DONE, etc. from Pi)
status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
status_sock.bind(("", STATUS_PORT))
status_sock.setblocking(False)

def poll_status() -> str | None:
    """Non-blocking check for status messages from the Pi."""
    try:
        data, _ = status_sock.recvfrom(1024)
        msg = json.loads(data.decode())
        return msg.get("status")
    except BlockingIOError:
        return None
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None

# ----------------------
# VIDEO STREAM (background thread)
# ----------------------
frame_size = WIDTH * HEIGHT * 3
_latest_frame = None
_frame_lock = threading.Lock()
proc = None

def _video_reader():
    """Continuously read frames from FFmpeg stdout in background."""
    global _latest_frame
    while proc and proc.poll() is None:
        raw = proc.stdout.read(frame_size)
        if len(raw) != frame_size:
            continue
        frame = np.frombuffer(raw, np.uint8).reshape((HEIGHT, WIDTH, 3)).copy()
        frame = cv2.flip(frame, -1)  # flip both axes (replaces Pi-side vflip+hflip)
        with _frame_lock:
            _latest_frame = frame

def get_latest_frame():
    """Non-blocking grab of the most recent video frame."""
    with _frame_lock:
        return _latest_frame

if ENABLE_VIDEO:
    sdp_file = f"stream_{STREAM_TYPE}.sdp"
    if os.path.exists(sdp_file):
        hello_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        hello_sock.sendto(b"HELLO", (RASPI_IP, HELLO_PORT))
        print("HELLO sent")
        time.sleep(1)

        ffmpeg_cmd = [
            "ffmpeg",
            "-protocol_whitelist", "file,udp,rtp",
            "-probesize", "32768",
            "-analyzeduration", "0",
            "-fflags", "nobuffer+discardcorrupt",
            "-flags", "low_delay",
            "-i", sdp_file,
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-"
        ]
        proc = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        threading.Thread(target=_video_reader, daemon=True).start()
        print("Receiving stream...")
    else:
        print(f"WARNING: {sdp_file} not found, running teleop-only mode")
        ENABLE_VIDEO = False
else:
    print("Video disabled, running teleop-only mode")

# ----------------------
# ARROW DETECTION (runs on laptop)
# Copied from cv.py — identical HSV logic
# ----------------------
LOWER_BLUE      = np.array([ 90,  30,  10])
UPPER_BLUE      = np.array([135, 255, 150])
MIN_BLUE_PIXELS = 50

def detect_arrow_in_frame(frame) -> str:
    """Return 'LEFT', 'RIGHT', 'UNCERTAIN', or 'NO ARROW'."""
    hsv         = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask        = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
    blue_pixels = np.where(mask > 0)

    if len(blue_pixels[1]) < MIN_BLUE_PIXELS:
        return "NO ARROW"

    xs          = blue_pixels[1]
    centroid_x  = (xs.min() + xs.max()) / 2.0
    left_count  = np.sum(xs <  centroid_x)
    right_count = np.sum(xs >= centroid_x)

    if left_count > right_count:
        return "LEFT"
    elif right_count > left_count:
        return "RIGHT"
    return "UNCERTAIN"

# ----------------------
# CV STATE MACHINE
# ----------------------
CV_OFF        = 0
CV_SCANNING   = 1
CV_TURNING    = 2
CV_WAIT_DRIVE = 3
CV_DRIVING    = 4

cv_state = CV_OFF
cv_consecutive = 0
cv_last_dir = None
cv_confirmed_dir = None
CONFIRM_FRAMES = 8

def cv_reset():
    global cv_state, cv_consecutive, cv_last_dir, cv_confirmed_dir
    cv_state = CV_OFF
    cv_consecutive = 0
    cv_last_dir = None
    cv_confirmed_dir = None

# ----------------------
# CONTROLS HELP
# ----------------------
print()
print("=== USAR TELEOP CONTROLS ===")
print("  W/A/S/D  = Forward / Left / Back / Right")
print("  Q/E      = Pivot left / Pivot right")
print("  [        = Servo left  (counter-clockwise)")
print("  ]        = Servo right (clockwise)")
print("  \\        = Servo center")
print("  1-5      = Speed level")
print("  B        = Toggle arrow detection overlay")
print("  C        = Start/stop CV mode")
print("  SPACE    = Emergency stop")
print("  ESC      = Quit")
print("============================")

# ----------------------
# MAIN LOOP: video + teleop
# ----------------------
DRIVE_KEYS = {
    ord('w'): "FWD",
    ord('s'): "BACK",
    ord('a'): "FWD_LEFT",
    ord('d'): "FWD_RIGHT",
    ord('q'): "LEFT",
    ord('e'): "RIGHT",
}

SERVO_KEYS = {
    ord('['): "SERVO_LEFT",
    ord(']'): "SERVO_RIGHT",
    ord('\\'): "SERVO_CENTER",
}

active_cmd = "STOP"
active_servo_cmd = None
detect_arrow = False
arrow_direction = ""

try:
    while True:
        # Grab latest frame
        frame = get_latest_frame() if ENABLE_VIDEO else None
        if frame is None:
            frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        # --- CV state machine ---
        if cv_state == CV_SCANNING:
            direction = detect_arrow_in_frame(frame)
            if direction in ("LEFT", "RIGHT"):
                if direction == cv_last_dir:
                    cv_consecutive += 1
                else:
                    cv_last_dir = direction
                    cv_consecutive = 1
            else:
                cv_consecutive = 0
                cv_last_dir = None

            # HUD for scanning
            confirmed = cv_consecutive >= CONFIRM_FRAMES
            hud_color = (0, 255, 0) if confirmed else (0, 165, 255)
            cv2.putText(frame, f"SCAN: {direction} ({cv_consecutive}/{CONFIRM_FRAMES})",
                        (10, HEIGHT - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hud_color, 2)
            cv2.putText(frame, "C = cancel",
                        (10, HEIGHT - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            if confirmed:
                cv_confirmed_dir = cv_last_dir
                turn_deg = 90.0 if cv_confirmed_dir == "LEFT" else -90.0
                print(f"Arrow confirmed: {cv_confirmed_dir} -> sending TURN {turn_deg:+.0f}")
                send_cmd("TURN", deg=turn_deg)
                cv_state = CV_TURNING

        elif cv_state == CV_TURNING:
            cv2.putText(frame, f"TURNING toward {cv_confirmed_dir} ball...",
                        (10, HEIGHT - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
            cv2.putText(frame, "C = cancel",
                        (10, HEIGHT - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            status = poll_status()
            if status == "TURN_DONE":
                print("Turn complete. Press SPACE to start driving.")
                cv_state = CV_WAIT_DRIVE
            elif status == "TURN_ERROR":
                print("Turn error!")
                cv_reset()

        elif cv_state == CV_WAIT_DRIVE:
            cv2.putText(frame, f"Turn done - facing {cv_confirmed_dir} ball",
                        (10, HEIGHT - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, "SPACE = start driving",
                        (10, HEIGHT - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            cv2.putText(frame, "C = cancel",
                        (10, HEIGHT - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        elif cv_state == CV_DRIVING:
            cv2.putText(frame, "DRIVING - heading hold active",
                        (10, HEIGHT - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, "SPACE = stop driving",
                        (10, HEIGHT - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

            status = poll_status()
            if status in ("DRIVE_STOPPED", "DRIVE_ERROR"):
                print(f"Drive ended: {status}")
                cv_reset()

        # --- Arrow detection overlay (independent of CV mode, toggled by 'b') ---
        if detect_arrow and cv_state == CV_OFF:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
            blue_pixels = np.where(mask > 0)
            if len(blue_pixels[1]) > 50:
                xs = blue_pixels[1]
                centroid_x = (xs.min() + xs.max()) / 2.0
                left_count = np.sum(xs < centroid_x)
                right_count = np.sum(xs >= centroid_x)
                if left_count > right_count:
                    arrow_direction = "< LEFT"
                elif right_count > left_count:
                    arrow_direction = "RIGHT >"
                else:
                    arrow_direction = "UNCERTAIN"
            else:
                arrow_direction = "NO ARROW"

        # --- HUD overlay ---
        speed_text = f"Speed: {current_power:.0f}/5"
        cmd_text = f"Cmd: {active_cmd}"
        cv2.putText(frame, speed_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, cmd_text, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if detect_arrow and arrow_direction and cv_state == CV_OFF:
            cv2.putText(frame, f"Arrow: {arrow_direction}", (WIDTH - 300, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # CV mode indicator
        if cv_state != CV_OFF:
            state_names = {CV_SCANNING: "SCANNING", CV_TURNING: "TURNING",
                           CV_WAIT_DRIVE: "READY", CV_DRIVING: "DRIVING"}
            cv2.putText(frame, f"CV: {state_names.get(cv_state, '?')}",
                        (WIDTH - 220, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "CV: OFF  [C]",
                        (WIDTH - 220, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

        cv2.namedWindow("USAR Teleop", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("USAR Teleop", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow("USAR Teleop", frame)

        key = cv2.waitKey(30) & 0xFF

        if key == 27:  # ESC
            if cv_state != CV_OFF:
                send_cmd("CV_STOP")
                cv_reset()
            send_cmd("STOP")
            break

        if key == ord('c'):
            if cv_state == CV_OFF:
                print("CV mode: SCANNING for arrow...")
                cv_state = CV_SCANNING
                cv_consecutive = 0
                cv_last_dir = None
            else:
                print("CV mode: CANCELLED")
                send_cmd("CV_STOP")
                cv_reset()

        elif key == ord(' '):
            if cv_state == CV_WAIT_DRIVE:
                print("Starting drive...")
                send_cmd("DRIVE_HEADING")
                cv_state = CV_DRIVING
            elif cv_state == CV_DRIVING:
                print("Stopping drive...")
                send_cmd("DRIVE_STOP")
                cv_reset()
            else:
                active_cmd = "STOP"
                active_servo_cmd = None
                if cv_state != CV_OFF:
                    send_cmd("CV_STOP")
                    cv_reset()
                print("STOP")

        elif cv_state == CV_OFF:
            # Normal teleop keys only active when CV is off
            if key in SPEED_LEVELS:
                current_power = SPEED_LEVELS[key]
                print(f"Speed set to {current_power:.0f}")
                active_servo_cmd = None

            elif key in SERVO_KEYS:
                active_servo_cmd = SERVO_KEYS[key]

            elif key in DRIVE_KEYS:
                active_cmd = DRIVE_KEYS[key]
                active_servo_cmd = None

            elif key == ord('b'):
                detect_arrow = not detect_arrow
                arrow_direction = ""
                print(f"Arrow detection: {'ON' if detect_arrow else 'OFF'}")

            else:
                active_servo_cmd = None

        # Send teleop commands only when CV is off
        if cv_state == CV_OFF:
            send_cmd(active_cmd)
            if active_servo_cmd:
                send_cmd(active_servo_cmd)

except KeyboardInterrupt:
    pass

if cv_state != CV_OFF:
    send_cmd("CV_STOP")
send_cmd("STOP")
if proc is not None:
    proc.terminate()
status_sock.close()
cv2.destroyAllWindows()
print("Client closed.")
