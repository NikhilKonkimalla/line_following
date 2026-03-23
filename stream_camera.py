#!/usr/bin/env python3
"""Camera streaming server - runs on the robot (Raspberry Pi).
Waits for a UDP HELLO from the client, then launches FFmpeg RTP stream.
"""
import socket
import subprocess

# ----------------------
# CONFIGURATION
# ----------------------
CAMERA_TYPE = "ps3"  # "ps3" or "logitech"

UDP_PORT = 7123
VIDEO_PORT = 5000
DEVICE = "/dev/video0"

WIDTH = 640
HEIGHT = 480
FPS = 15

INPUT_FORMATS = {
    "logitech": "mjpeg",
    "ps3": "yuyv422"
}

INPUT_FORMAT = INPUT_FORMATS[CAMERA_TYPE]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))

print("Waiting for client HELLO...")
while True:
    data, client_addr = sock.recvfrom(1024)
    if data.strip() == b"HELLO":
        break
    print(f"Ignoring unexpected packet from {client_addr[0]}")

client_ip = client_addr[0]
print("Client connected:", client_ip)

cmd = [
    "ffmpeg",

    "-f", "v4l2",
    "-input_format", INPUT_FORMAT,
    "-video_size", f"{WIDTH}x{HEIGHT}",
    "-framerate", str(FPS),
    "-i", DEVICE,

    "-vcodec", "libx264",
    "-preset", "ultrafast",
    "-tune", "zerolatency",
    "-vf", "vflip,hflip",
    "-g", "30",
    "-pix_fmt", "yuv420p",

    "-f", "rtp",
    "-sdp_file", f"stream_{CAMERA_TYPE}.sdp",
    f"rtp://{client_ip}:{VIDEO_PORT}"
]

print("Starting video stream...")
subprocess.run(cmd)
