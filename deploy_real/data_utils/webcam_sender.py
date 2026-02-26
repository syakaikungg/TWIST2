#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Webcam/Test-pattern ZMQ Sender

OrinVideoSender と同じフォーマットで映像を ZMQ PUB ソケットから送信する。
ウェブカメラがあればウェブカメラ映像を、なければテストパターンを生成して送信する。

フォーマット: [4-byte width][4-byte height][4-byte JPEG length][JPEG data]

使い方:
    # テストパターン（カメラなし）
    python webcam_sender.py --port 5555

    # ウェブカメラ指定
    python webcam_sender.py --port 5555 --camera 0

    # VisionClient で受信テスト（別ターミナル）
    python vision_client.py  # server_address="127.0.0.1", port=5555
"""

import argparse
import struct
import time
import sys

import cv2
import numpy as np
import zmq


def generate_test_frame(width, height, frame_count, fps):
    """テストパターンフレームを生成"""
    frame = np.zeros((height, width, 3), dtype=np.uint8)

    # 色のグラデーション背景
    t = frame_count / max(fps, 1)
    for y in range(height):
        r = int(127 + 127 * np.sin(2 * np.pi * y / height + t))
        g = int(127 + 127 * np.sin(2 * np.pi * y / height + t + 2.094))
        b = int(127 + 127 * np.sin(2 * np.pi * y / height + t + 4.189))
        frame[y, :] = (b, g, r)

    # 動く円
    cx = int(width / 2 + width / 4 * np.sin(t * 0.5))
    cy = int(height / 2 + height / 4 * np.cos(t * 0.7))
    cv2.circle(frame, (cx, cy), 40, (0, 255, 0), -1)

    # フレーム情報テキスト
    text = f"TEST PATTERN  Frame: {frame_count}  FPS: {fps}"
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"Time: {t:.1f}s", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"Resolution: {width}x{height}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # タイムスタンプバー（映像が動いていることの確認用）
    bar_x = int((frame_count % width))
    cv2.line(frame, (bar_x, height - 20), (bar_x, height), (0, 0, 255), 3)

    return frame


def main():
    parser = argparse.ArgumentParser(description="Webcam/Test-pattern ZMQ sender")
    parser.add_argument("--port", type=int, default=5555, help="ZMQ PUB port (default: 5555)")
    parser.add_argument("--camera", type=int, default=-1,
                        help="Camera device index. -1 = test pattern (default: -1)")
    parser.add_argument("--width", type=int, default=640, help="Frame width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="Frame height (default: 480)")
    parser.add_argument("--fps", type=int, default=30, help="Target FPS (default: 30)")
    parser.add_argument("--jpeg_quality", type=int, default=80, help="JPEG quality 0-100 (default: 80)")
    args = parser.parse_args()

    # --- カメラ or テストパターン ---
    cap = None
    use_camera = False

    if args.camera >= 0:
        cap = cv2.VideoCapture(args.camera)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            use_camera = True
            print(f"[Sender] Webcam opened: device {args.camera}")
        else:
            print(f"[Sender] Cannot open camera {args.camera}, falling back to test pattern")
            cap.release()
            cap = None

    if not use_camera:
        print(f"[Sender] Using test pattern ({args.width}x{args.height} @ {args.fps} FPS)")

    # --- ZMQ PUB setup ---
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    bind_addr = f"tcp://0.0.0.0:{args.port}"
    socket.bind(bind_addr)
    print(f"[Sender] ZMQ PUB bound to {bind_addr}")
    print(f"[Sender] Format: [width:4B][height:4B][jpeg_len:4B][jpeg_data]")
    print(f"[Sender] Press Ctrl+C to stop\n")

    # JPEG encode params
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality]

    frame_count = 0
    interval = 1.0 / args.fps
    t_start = time.time()

    try:
        while True:
            t_loop = time.time()

            # フレーム取得
            if use_camera:
                ret, frame = cap.read()
                if not ret:
                    print("[Sender] Camera read failed, skipping")
                    time.sleep(0.01)
                    continue
            else:
                frame = generate_test_frame(args.width, args.height, frame_count, args.fps)

            h, w = frame.shape[:2]

            # JPEG エンコード
            success, jpeg_buf = cv2.imencode('.jpg', frame, encode_param)
            if not success:
                continue
            jpeg_bytes = jpeg_buf.tobytes()
            jpeg_len = len(jpeg_bytes)

            # メッセージ組み立て: [width][height][jpeg_length][jpeg_data]
            header = struct.pack('iii', w, h, jpeg_len)
            message = header + jpeg_bytes

            # 送信
            socket.send(message, zmq.NOBLOCK)

            frame_count += 1

            # FPS 表示 (毎秒)
            if frame_count % args.fps == 0:
                elapsed = time.time() - t_start
                actual_fps = frame_count / elapsed if elapsed > 0 else 0
                print(f"[Sender] Sent {frame_count} frames | "
                      f"FPS: {actual_fps:.1f} | "
                      f"JPEG size: {jpeg_len:,} bytes | "
                      f"Resolution: {w}x{h}")

            # FPS制限
            dt = time.time() - t_loop
            sleep_time = interval - dt
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print(f"\n[Sender] Stopped. Total frames sent: {frame_count}")
    finally:
        if cap is not None:
            cap.release()
        socket.close()
        context.term()


if __name__ == "__main__":
    main()
