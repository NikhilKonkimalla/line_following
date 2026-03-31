#!/usr/bin/env python3
"""USAR Client - runs on your laptop.
Combines keyboard teleoperation with optional video streaming.
Pressing 'c' toggles live arrow detection on the streamed camera feed.
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

# Status listener (receives DRIVE_DONE, TURN_DONE, etc. from Pi)
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
# Simple HSV threshold + centroid pixel weighting
# ----------------------
MIN_CARD_AREA    = 500
WHITE_V_THRESH   = 200   # V channel threshold for detecting bright white card
SAMPLE_FRAMES    = 100   # frames to accumulate before locking in a direction
UNCERTAIN_MARGIN = 0.5  # if vote split is within this ratio, consider uncertain
V_THRESH_STEP    = 20    # how much to lower V threshold on retry
V_THRESH_MIN     = 120   # don't go below this
_debug_mask      = None
_left_votes      = 0
_right_votes     = 0
_sample_count    = 0
_locked_result   = None
_retry_state     = 0     # 0=scanning, 1=waiting 2s, 2=retrying with lower thresh
_retry_time      = 0.0
_current_v_thresh = WHITE_V_THRESH

def detect_arrow_in_frame(frame) -> str:
    """Two-stage arrow detection with temporal averaging and auto-retry.
    Stage 1: Find the white card (largest bright blob).
    Stage 2: Count white pixels left vs right in the card ROI.
    Accumulates votes over SAMPLE_FRAMES. If uncertain, waits 2s then
    retries with a lower V threshold to pick up more washed-out blues.
    """
    global _debug_mask, _left_votes, _right_votes, _sample_count
    global _locked_result, _retry_state, _retry_time, _current_v_thresh

    # If locked with a definite answer, return it
    if _locked_result is not None:
        return _locked_result

    # If in the 2-second wait before retry
    if _retry_state == 1:
        elapsed = time.monotonic() - _retry_time
        if elapsed < 2.0:
            return f"UNCERTAIN - retrying in {2.0 - elapsed:.1f}s"
        # Time's up — lower threshold and restart scanning
        _current_v_thresh = max(V_THRESH_MIN, _current_v_thresh - V_THRESH_STEP)
        _left_votes = 0
        _right_votes = 0
        _sample_count = 0
        _retry_state = 2
        print(f"Retrying with V_THRESH={_current_v_thresh}")

    # --- Stage 1: Find the card ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    v_channel = hsv[:, :, 2]
    card_mask = cv2.inRange(v_channel, np.array([_current_v_thresh]), np.array([255]))

    contours, _ = cv2.findContours(card_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        _debug_mask = card_mask
        return f"SCANNING ({_sample_count}/{SAMPLE_FRAMES})"

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_CARD_AREA:
        _debug_mask = card_mask
        return f"SCANNING ({_sample_count}/{SAMPLE_FRAMES})"

    rx, ry, rw, rh = cv2.boundingRect(largest)

    # --- Stage 2: Raw white pixel analysis within the card ROI ---
    roi_v = v_channel[ry:ry+rh, rx:rx+rw]
    roi_mask = (roi_v >= _current_v_thresh).astype(np.uint8) * 255
    _debug_mask = roi_mask

    center_x = rw // 2
    left_white = np.count_nonzero(roi_mask[:, :center_x])
    right_white = np.count_nonzero(roi_mask[:, center_x:])

    total = left_white + right_white
    if total == 0:
        return f"SCANNING ({_sample_count}/{SAMPLE_FRAMES})"

    # Cast a vote
    if left_white < right_white:
        _left_votes += 1
    elif right_white < left_white:
        _right_votes += 1
    _sample_count += 1

    # Check if we've collected enough samples
    if _sample_count >= SAMPLE_FRAMES:
        total_votes = _left_votes + _right_votes
        if total_votes == 0:
            margin = 0.0
        else:
            margin = abs(_left_votes - _right_votes) / total_votes

        if margin > UNCERTAIN_MARGIN:
            # Clear result
            if _left_votes > _right_votes:
                _locked_result = "< LEFT"
            else:
                _locked_result = "RIGHT >"
            print(f"LOCKED: {_locked_result} (L:{_left_votes} R:{_right_votes} / {_sample_count} frames, margin={margin:.2f})")
            return _locked_result
        else:
            # Too close to call — trigger retry with lower threshold
            if _current_v_thresh - V_THRESH_STEP >= V_THRESH_MIN:
                print(f"UNCERTAIN (L:{_left_votes} R:{_right_votes}, margin={margin:.2f}) — retrying in 2s")
                _retry_state = 1
                _retry_time = time.monotonic()
                return "UNCERTAIN - retrying in 2.0s"
            else:
                # Already at minimum threshold, lock as uncertain
                _locked_result = "UNCERTAIN"
                print(f"LOCKED: UNCERTAIN (L:{_left_votes} R:{_right_votes}, already at min V_THRESH)")
                return _locked_result

    return f"SCANNING ({_sample_count}/{SAMPLE_FRAMES})"

def reset_arrow_detection():
    """Reset the vote accumulator so detection can run again."""
    global _left_votes, _right_votes, _sample_count, _locked_result
    global _retry_state, _retry_time, _current_v_thresh
    _left_votes = 0
    _right_votes = 0
    _sample_count = 0
    _locked_result = None
    _retry_state = 0
    _retry_time = 0.0
    _current_v_thresh = WHITE_V_THRESH

# ----------------------
# CV SEQUENCE STATE MACHINE
# ----------------------
# Press C → WAIT(7s) → DRIVE1(6in) → DETECT(arrow) → DRIVE2(6in) → TURN(direction) → DONE
CV_OFF       = 0
CV_WAIT      = 1   # 7-second countdown
CV_DRIVE1    = 2   # drive forward 6 inches
CV_DETECT    = 3   # run arrow detection (100 frames)
CV_DRIVE2    = 4   # drive forward 6 more inches
CV_DRIVE3    = 5   # drive forward 6 more inches (guaranteed before turn)
CV_TURN      = 6   # rotate toward arrow direction
CV_DONE      = 7

SIX_INCHES_CM = 15.24
TURN_DEG      = 90.0

cv_state     = CV_OFF
cv_direction = ""
cv_start_time = 0.0

# ----------------------
# CONTROLS HELP
# ----------------------
print()
print("=== USAR TELEOP CONTROLS ===")
print("  W/A/S/D  = Forward / Left / Back / Right")
print("  Q/E      = Pivot left / Pivot right")
print("  [        = Arm forward  (stepper CW, hold)")
print("  ]        = Arm backward (stepper CCW, hold)")
print("  \\        = Arm stop / release coils")
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

        # --- CV sequence state machine ---
        if cv_state == CV_WAIT:
            remaining = 7.0 - (time.monotonic() - cv_start_time)
            if remaining > 0:
                cv2.putText(frame, f"CV starting in {remaining:.1f}s",
                            (10, HEIGHT - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
            else:
                print("Driving forward 6 inches...")
                send_cmd("DRIVE_TOF", distance_cm=SIX_INCHES_CM)
                cv_state = CV_DRIVE1

        elif cv_state == CV_DRIVE1:
            cv2.putText(frame, "DRIVING 6 inches...",
                        (10, HEIGHT - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
            status = poll_status()
            if status == "DRIVE_DONE":
                print("Drive 1 done. Starting arrow detection...")
                reset_arrow_detection()
                cv_state = CV_DETECT
            elif status == "DRIVE_ERROR":
                print("Drive error!")
                cv_state = CV_OFF

        elif cv_state == CV_DETECT:
            cv_direction = detect_arrow_in_frame(frame)
            color = (0, 255, 0) if "LEFT" in cv_direction or "RIGHT" in cv_direction else (0, 165, 255)
            cv2.putText(frame, f"Arrow: {cv_direction}",
                        (10, HEIGHT - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            # Debug mask
            if _debug_mask is not None:
                mask_small = cv2.resize(_debug_mask, (160, 120))
                mask_bgr = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
                frame[0:120, WIDTH - 160:WIDTH] = mask_bgr
            # Check if detection locked
            if _locked_result is not None and _locked_result != "UNCERTAIN":
                print(f"Arrow detected: {_locked_result}. Driving forward 6 more inches...")
                send_cmd("DRIVE_TOF", distance_cm=SIX_INCHES_CM)
                cv_state = CV_DRIVE2
            elif _locked_result == "UNCERTAIN":
                print("Arrow uncertain. Driving forward 6 inches anyway...")
                send_cmd("DRIVE_TOF", distance_cm=SIX_INCHES_CM)
                cv_state = CV_DRIVE2

        elif cv_state == CV_DRIVE2:
            cv2.putText(frame, f"DRIVING 6 inches... (arrow: {_locked_result})",
                        (10, HEIGHT - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
            status = poll_status()
            if status is not None:
                print(f"[CV_DRIVE2] Got status: {status}")
            if status == "DRIVE_DONE":
                print("Drive 2 done. Driving 6 more inches before turn...")
                send_cmd("DRIVE_TOF", distance_cm=SIX_INCHES_CM)
                cv_state = CV_DRIVE3
            elif status == "DRIVE_ERROR":
                print("Drive 2 error!")
                cv_state = CV_OFF

        elif cv_state == CV_DRIVE3:
            cv2.putText(frame, f"DRIVING 6 inches (pre-turn)... (arrow: {_locked_result})",
                        (10, HEIGHT - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
            status = poll_status()
            if status == "DRIVE_DONE":
                if _locked_result and "LEFT" in _locked_result:
                    turn_deg = TURN_DEG
                    print(f"Drive 3 done. Turning LEFT {turn_deg} deg...")
                elif _locked_result and "RIGHT" in _locked_result:
                    turn_deg = -TURN_DEG
                    print(f"Drive 3 done. Turning RIGHT {turn_deg} deg...")
                else:
                    print("Drive 3 done. No clear direction — skipping turn.")
                    cv_state = CV_DONE
                    cv_start_time = time.monotonic()
                    turn_deg = None
                if turn_deg is not None:
                    send_cmd("TURN", deg=turn_deg)
                    cv_state = CV_TURN
            elif status == "DRIVE_ERROR":
                print("Drive 3 error!")
                cv_state = CV_OFF

        elif cv_state == CV_TURN:
            cv2.putText(frame, f"TURNING {_locked_result}...",
                        (10, HEIGHT - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
            status = poll_status()
            if status in ("TURN_DONE", "TURN_ERROR"):
                print(f"Turn complete: {status}")
                cv_state = CV_DONE
                cv_start_time = time.monotonic()

        elif cv_state == CV_DONE:
            cv2.putText(frame, "SEQUENCE COMPLETE",
                        (10, HEIGHT - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            if time.monotonic() - cv_start_time > 3.0:
                cv_state = CV_OFF

        # --- Arrow detection overlay (independent of CV mode, toggled by 'b') ---
        if detect_arrow and cv_state == CV_OFF:
            arrow_direction = detect_arrow_in_frame(frame)

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
            state_names = {CV_WAIT: "WAIT", CV_DRIVE1: "DRIVE1", CV_DETECT: "DETECT",
                           CV_DRIVE2: "DRIVE2", CV_DRIVE3: "DRIVE3", CV_TURN: "TURN", CV_DONE: "DONE"}
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
            send_cmd("STOP")
            break

        if key == ord('c'):
            if cv_state == CV_OFF:
                print("CV sequence started — waiting 7 seconds...")
                cv_state = CV_WAIT
                cv_start_time = time.monotonic()
                cv_direction = ""
                reset_arrow_detection()
            else:
                print("CV sequence CANCELLED")
                send_cmd("CV_STOP")
                cv_state = CV_OFF

        elif key == ord(' '):
            active_cmd = "STOP"
            active_servo_cmd = None
            if cv_state != CV_OFF:
                send_cmd("CV_STOP")
                cv_state = CV_OFF
            print("STOP")

        elif cv_state == CV_OFF:
            # Normal teleop keys only active when CV is off
            if key in SPEED_LEVELS:
                current_power = SPEED_LEVELS[key]
                print(f"Speed set to {current_power:.0f}")

            elif key in SERVO_KEYS:
                active_servo_cmd = SERVO_KEYS[key]

            elif key in DRIVE_KEYS:
                active_cmd = DRIVE_KEYS[key]

            elif key == ord('b'):
                detect_arrow = not detect_arrow
                arrow_direction = ""
                print(f"Arrow detection: {'ON' if detect_arrow else 'OFF'}")

            elif key != 0xFF:
                # A key was pressed that isn't a known command — clear servo
                active_servo_cmd = None

        # Send teleop commands only when CV sequence is not running
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
