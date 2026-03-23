#!/usr/bin/env python3
"""USAR Client - runs on your laptop.
Combines keyboard teleoperation with optional video streaming.
If camera is not set up, runs teleop-only mode with a blank HUD window.
"""
import socket
import subprocess
import os
import numpy as np
import cv2
import time
import json

# ----------------------
# CONFIGURATION - UPDATE THESE
# ----------------------
RASPI_IP = "172.26.230.146"  # <-- change to your robot's IP
HELLO_PORT = 7123
TELEOP_PORT = 7124

WIDTH = 640
HEIGHT = 480

STREAM_TYPE = "ps3"  # "ps3" or "logitech"
ENABLE_VIDEO = False  # set True once camera is set up

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

def send_cmd(cmd: str):
    msg = json.dumps({"cmd": cmd, "power": current_power})
    teleop_sock.sendto(msg.encode(), (RASPI_IP, TELEOP_PORT))

# ----------------------
# VIDEO STREAM (optional)
# ----------------------
proc = None
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
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-i", sdp_file,
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-"
        ]
        proc = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        print("Receiving stream...")
    else:
        print(f"WARNING: {sdp_file} not found, running teleop-only mode")
        ENABLE_VIDEO = False
else:
    print("Video disabled, running teleop-only mode")

print()
print("=== USAR TELEOP CONTROLS ===")
print("  W/A/S/D  = Forward / Left / Back / Right")
print("  Q/E      = Pivot left / Pivot right")
print("  1-5      = Speed level")
print("  SPACE    = Emergency stop")
print("  ESC      = Quit")
print("============================")

# ----------------------
# MAIN LOOP: video + teleop
# ----------------------
frame_size = WIDTH * HEIGHT * 3

DRIVE_KEYS = {
    ord('w'): "FWD",
    ord('s'): "BACK",
    ord('a'): "FWD_LEFT",
    ord('d'): "FWD_RIGHT",
    ord('q'): "LEFT",
    ord('e'): "RIGHT",
}

active_cmd = "STOP"

try:
    while True:
        if ENABLE_VIDEO and proc is not None:
            raw_frame = proc.stdout.read(frame_size)
            if len(raw_frame) != frame_size:
                if proc.poll() is not None:
                    print("Video stream ended.")
                    break
                continue
            frame = np.frombuffer(raw_frame, np.uint8).reshape((HEIGHT, WIDTH, 3)).copy()
        else:
            # Teleop-only: black frame with HUD
            frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        # Draw HUD overlay
        speed_text = f"Speed: {current_power:.0f}/5"
        cmd_text = f"Cmd: {active_cmd}"
        cv2.putText(frame, speed_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, cmd_text, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("USAR Teleop", frame)

        key = cv2.waitKey(30) & 0xFF  # 30ms ~33fps for teleop-only mode

        if key == 27:  # ESC
            send_cmd("STOP")
            break

        if key in SPEED_LEVELS:
            current_power = SPEED_LEVELS[key]
            print(f"Speed set to {current_power:.0f}")

        elif key in DRIVE_KEYS:
            active_cmd = DRIVE_KEYS[key]

        elif key == ord(' '):
            active_cmd = "STOP"
            print("STOP")

        send_cmd(active_cmd)

except KeyboardInterrupt:
    pass

send_cmd("STOP")
if proc is not None:
    proc.terminate()
cv2.destroyAllWindows()
print("Client closed.")
