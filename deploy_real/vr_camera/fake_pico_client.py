#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PICO の XRobot アプリの代わりに g1_cam_to_pico.py を試すための疑似クライアント。

  1. 受信用ポートを開く
  2. サーバー (既定 127.0.0.1:13579) に OPEN_CAMERA を送る
  3. 届いた [4B 長さ][H.264] を数えて、ファイルに保存する

例 (G1 上で、外に出さずに試す):
  python3 fake_pico_client.py --server 127.0.0.1:13579 --size 2560x720 --seconds 5
"""

import argparse
import socket
import struct
import sys
import time


def build_open_camera(width, height, fps, bitrate, hevc, port, camera, ip):
    cam = camera.encode()
    ipb = ip.encode()
    data = bytes([0xCA, 0xFE, 1]) + struct.pack("<7i", width, height, fps, bitrate, hevc, 0, port)
    data += bytes([len(cam)]) + cam + bytes([len(ipb)]) + ipb
    cmd = b"OPEN_CAMERA"
    body = struct.pack("<I", len(cmd)) + cmd + struct.pack("<I", len(data)) + data
    return struct.pack(">I", len(body)) + body


def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return buf


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--server", default="127.0.0.1:13579")
    p.add_argument("--my-ip", default="127.0.0.1", help="サーバーから見たこのクライアントの IP")
    p.add_argument("--port", type=int, default=12345, help="映像を受けるポート")
    p.add_argument("--size", default="2560x720")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--bitrate", type=int, default=4000000)
    p.add_argument("--camera", default="ZEDMINI")
    p.add_argument("--seconds", type=float, default=5)
    p.add_argument("--out", default="/tmp/fake_pico_recv.h264")
    a = p.parse_args()
    w, h = [int(x) for x in a.size.lower().split("x")]

    rx = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", a.port))
    rx.listen(1)
    rx.settimeout(15)

    host, port = a.server.rsplit(":", 1)
    ctl = socket.create_connection((host, int(port)), timeout=5)
    ctl.sendall(build_open_camera(w, h, a.fps, a.bitrate, 0, a.port, a.camera, a.my_ip))
    print("OPEN_CAMERA を送りました %dx%d -> 受信 %s:%d" % (w, h, a.my_ip, a.port), flush=True)

    conn, addr = rx.accept()
    print("映像の接続 from %s:%d" % addr, flush=True)
    conn.settimeout(5)
    n = 0
    total = 0
    t0 = time.time()
    first_t = None
    with open(a.out, "wb") as f:
        try:
            while time.time() - t0 < a.seconds:
                ln = struct.unpack(">I", recv_exact(conn, 4))[0]
                if ln <= 0 or ln > 20 * 1024 * 1024:
                    print("おかしな長さ %d" % ln)
                    return 1
                data = recv_exact(conn, ln)
                if first_t is None:
                    first_t = time.time()
                    print("最初のフレーム %.2f 秒後, %d bytes, head=%s" % (
                        first_t - t0, ln, data[:8].hex()), flush=True)
                f.write(data)
                n += 1
                total += ln
        except Exception as e:
            print("受信終了: %s" % e)
    dt = time.time() - (first_t or t0)
    print("受信 %d フレーム, %.1f fps, %.0f kbps -> %s" % (n, n / max(dt, 1e-3), total * 8 / max(dt, 1e-3) / 1000, a.out))
    conn.close()
    ctl.close()
    return 0 if n > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
