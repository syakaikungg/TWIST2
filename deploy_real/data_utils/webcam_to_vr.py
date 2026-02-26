#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Webcam to VR Streamer (RobotVision-PC / OrinVideoSender compatible)

ウェブカメラの映像を H.264 エンコードして PICO VR の XRobot APP に送信する。
RobotVision-PC の H264Encoder と同じ方式 (PyAV = FFmpeg libavcodec) を使用。

プロトコル:
  - TCP で [4-byte big-endian length][H264 Annex B data] を送信
  - 1パケット = 1フレーム (SPS+PPS+IDR または P-slice)

使い方:
  # Listen モード (推奨: PICO VR の UI で PC の IP を入力)
  python webcam_to_vr.py --listen 0.0.0.0:13579 --camera 0

  # Direct モード (PICO で先に Listen を押してから)
  python webcam_to_vr.py --send --server 192.168.123.75 --port 12345 --camera 0
"""

import argparse
import socket
import struct
import sys
import threading
import time
import traceback
import os
from fractions import Fraction

import numpy as np

try:
    import av
    HAS_PYAV = True
except ImportError:
    HAS_PYAV = False
    print("[Warning] PyAV not found. Install with: pip install av")


def send_h264_packet(sock: socket.socket, data: bytes):
    """[4-byte big-endian length][data] 形式で TCP 送信
    
    RobotVision-PC の encoder_callback と同じ:
        packet[0] = (size >> 24) & 0xFF;
        packet[1] = (size >> 16) & 0xFF;
        packet[2] = (size >> 8) & 0xFF;
        packet[3] = (size) & 0xFF;
        std::copy(data, data + size, packet.begin() + 4);
    """
    header = struct.pack('>I', len(data))
    sock.sendall(header + data)


def parse_open_camera(data: bytes):
    """OPEN_CAMERA コマンドのバイナリデータをパース"""
    try:
        if len(data) < 4:
            return None
        body_len = struct.unpack('>I', data[:4])[0]
        body = data[4:4 + body_len]

        if len(body) < 8:
            return None
        cmd_len = struct.unpack('<I', body[:4])[0]
        command = body[4:4 + cmd_len].decode('utf-8', errors='replace').rstrip('\x00')
        data_offset = 4 + cmd_len
        data_len = struct.unpack('<I', body[data_offset:data_offset + 4])[0]
        cam_data = body[data_offset + 4:data_offset + 4 + data_len]

        print(f"[Listen] Command: '{command}', data_len: {data_len}")

        if command != "OPEN_CAMERA":
            return None

        if len(cam_data) < 31:
            return None
        if cam_data[0] != 0xCA or cam_data[1] != 0xFE:
            print("[Listen] Invalid magic bytes")
            return None

        offset = 3  # skip magic + version
        width = struct.unpack('<i', cam_data[offset:offset + 4])[0]; offset += 4
        height = struct.unpack('<i', cam_data[offset:offset + 4])[0]; offset += 4
        fps = struct.unpack('<i', cam_data[offset:offset + 4])[0]; offset += 4
        bitrate = struct.unpack('<i', cam_data[offset:offset + 4])[0]; offset += 4
        enable_hevc = struct.unpack('<i', cam_data[offset:offset + 4])[0]; offset += 4
        render_mode = struct.unpack('<i', cam_data[offset:offset + 4])[0]; offset += 4
        port = struct.unpack('<i', cam_data[offset:offset + 4])[0]; offset += 4

        cam_str_len = cam_data[offset]; offset += 1
        camera = cam_data[offset:offset + cam_str_len].decode('utf-8') if cam_str_len > 0 else ""
        offset += cam_str_len

        ip_str_len = cam_data[offset]; offset += 1
        ip = cam_data[offset:offset + ip_str_len].decode('utf-8') if ip_str_len > 0 else ""

        config = {
            'width': width, 'height': height, 'fps': fps,
            'bitrate': bitrate, 'enable_hevc': enable_hevc,
            'render_mode': render_mode, 'port': port,
            'camera': camera, 'ip': ip
        }
        print(f"[Listen] Camera config: {config}")
        return config

    except Exception as e:
        print(f"[Listen] Parse error: {e}")
        traceback.print_exc()
        return None


def stream_webcam_pyav(target_ip: str, target_port: int, camera_idx: int,
                       width: int, height: int, fps: int, bitrate: int):
    """PyAV (FFmpeg libavcodec) でウェブカメラ映像を H.264 エンコードして TCP 送信
    
    RobotVision-PC の H264Encoder.cpp と同じ方式:
    - avcodec_receive_packet → 1パケット = 1フレーム
    - Annex B format with start codes
    - baseline profile, ultrafast preset, zerolatency tune
    """

    cam_fps = min(fps, 30)

    print(f"\n{'='*60}")
    print(f"Webcam → VR Streaming (PyAV)")
    print(f"  Camera: /dev/video{camera_idx}")
    print(f"  Output: {width}x{height} @ {cam_fps}fps")
    print(f"  Bitrate: {bitrate // 1000}kbps")
    print(f"  Target: {target_ip}:{target_port}")
    print(f"  Profile: Baseline (annexb)")
    print(f"{'='*60}\n")

    # まずカメラを開く (接続前に準備完了させる)
    print(f"[Stream] Opening camera /dev/video{camera_idx}...")
    try:
        input_container = av.open(
            f'/dev/video{camera_idx}',
            format='v4l2',
            options={
                'framerate': str(cam_fps),
                'video_size': '640x480',
            }
        )
    except Exception as e:
        print(f"[Stream] Failed to open camera: {e}")
        return

    input_stream = input_container.streams.video[0]
    print(f"[Stream] Camera opened: {input_stream.width}x{input_stream.height} @ {input_stream.average_rate}fps")

    # TCP 接続 (カメラ準備完了後)
    print(f"[Stream] Connecting to {target_ip}:{target_port}...")
    retry = 10
    sock = None
    while retry > 0:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((target_ip, target_port))
            sock.settimeout(None)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[Stream] Connected!")
            break
        except Exception as e:
            print(f"[Stream] Connection failed ({retry} retries left): {e}")
            retry -= 1
            if sock:
                sock.close()
                sock = None
            time.sleep(1)

    if sock is None:
        print("[Stream] Failed to connect")
        input_container.close()
        return

    # ステレオ side-by-side が必要か判定
    need_stereo = (width >= 2000)  # 2560x720 or 2160x810
    if need_stereo:
        half_w = width // 2
        half_h = height
        print(f"[Stream] Stereo mode: {half_w}x{half_h} x2 → {width}x{height}")
    else:
        half_w = width
        half_h = height

    # H.264 エンコーダーを作成
    # RobotVision-PC の H264Encoder と同じ設定
    codec = av.Codec('libx264', 'w')
    encoder = codec.create()
    encoder.width = width
    encoder.height = height
    encoder.pix_fmt = 'yuv420p'
    encoder.time_base = Fraction(1, cam_fps)
    encoder.framerate = Fraction(cam_fps, 1)
    encoder.bit_rate = bitrate
    encoder.max_b_frames = 0  # B-frames を無効化
    encoder.gop_size = cam_fps * 2  # 2秒ごとにキーフレーム

    # x264 固有オプション (RobotVision-PC と同じ)
    encoder.options = {
        'preset': 'ultrafast',
        'tune': 'zerolatency',
        'profile': 'baseline',
        'annexb': '1',          # Annex B start codes
        'sc_threshold': '0',    # Scene change threshold off
    }

    encoder.open()
    print(f"[Stream] Encoder opened: {encoder.width}x{encoder.height} {encoder.bit_rate}bps")

    frame_count = 0
    total_bytes = 0
    t_start = time.time()

    try:
        for raw_frame in input_container.decode(input_stream):
            # RGB24 に変換
            rgb_frame = raw_frame.reformat(format='rgb24')
            frame_np = rgb_frame.to_ndarray()

            # リサイズ
            # PyAV の reformat は resize をサポートしないので numpy で行う
            # ただし簡易リサイズ（cv2 があれば使う、なければ nearest neighbor）
            if frame_np.shape[1] != half_w or frame_np.shape[0] != half_h:
                try:
                    import cv2
                    frame_np = cv2.resize(frame_np, (half_w, half_h))
                except ImportError:
                    # Nearest neighbor resize using numpy
                    h_ratio = half_h / frame_np.shape[0]
                    w_ratio = half_w / frame_np.shape[1]
                    h_idx = (np.arange(half_h) / h_ratio).astype(int)
                    w_idx = (np.arange(half_w) / w_ratio).astype(int)
                    frame_np = frame_np[np.ix_(h_idx, w_idx)]

            # ステレオ side-by-side
            if need_stereo:
                frame_np = np.concatenate([frame_np, frame_np], axis=1)

            # PyAV VideoFrame に変換
            out_frame = av.VideoFrame.from_ndarray(frame_np, format='rgb24')
            out_frame.pts = frame_count
            out_frame.time_base = Fraction(1, cam_fps)

            # H.264 エンコード → 1パケット/フレーム
            packets = encoder.encode(out_frame)
            for pkt in packets:
                pkt_bytes = bytes(pkt)
                if len(pkt_bytes) == 0:
                    continue

                # デバッグ: 最初の数フレームのパケット情報を表示
                if frame_count < 5:
                    hex_preview = pkt_bytes[:32].hex(' ')
                    nal_type = -1
                    # Start code を探す
                    if pkt_bytes[:4] == b'\x00\x00\x00\x01':
                        nal_type = pkt_bytes[4] & 0x1F
                    elif pkt_bytes[:3] == b'\x00\x00\x01':
                        nal_type = pkt_bytes[3] & 0x1F
                    print(f"[Stream] Frame {frame_count}: {len(pkt_bytes)} bytes, "
                          f"NAL type={nal_type}, key={pkt.is_keyframe}")
                    print(f"  First 32 bytes: {hex_preview}")

                try:
                    send_h264_packet(sock, pkt_bytes)
                    frame_count += 1
                    total_bytes += len(pkt_bytes)
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                    print(f"\n[Stream] Connection lost: {e}")
                    return

            # 進捗表示
            elapsed = time.time() - t_start
            if frame_count > 0 and frame_count % (cam_fps * 2) == 0:
                fps_actual = frame_count / elapsed
                avg_size = total_bytes / frame_count / 1024
                print(f"[Stream] {frame_count} frames | "
                      f"{total_bytes / 1024 / 1024:.1f} MB | "
                      f"FPS: {fps_actual:.1f} | "
                      f"Avg: {avg_size:.1f} KB/f | "
                      f"Elapsed: {elapsed:.0f}s")

    except KeyboardInterrupt:
        print("\n[Stream] Interrupted")
    except Exception as e:
        print(f"\n[Stream] Error: {e}")
        traceback.print_exc()
    finally:
        # Flush encoder
        try:
            packets = encoder.encode(None)
            for pkt in packets:
                pkt_bytes = bytes(pkt)
                if pkt_bytes:
                    send_h264_packet(sock, pkt_bytes)
        except:
            pass

        input_container.close()
        sock.close()
        elapsed = time.time() - t_start
        print(f"[Stream] Stopped. {frame_count} frames, "
              f"{total_bytes / 1024 / 1024:.1f} MB in {elapsed:.1f}s")


def listen_mode(listen_addr: str, camera_idx: int, width: int, height: int,
                fps: int, bitrate: int):
    """Listen モード: PICO からの OPEN_CAMERA コマンドを待つ"""
    host, port_str = listen_addr.rsplit(':', 1)
    port = int(port_str)

    print(f"\n{'='*60}")
    print(f"Webcam to VR - Listen Mode (PyAV)")
    print(f"  Listening on: {host}:{port}")
    print(f"  Camera: /dev/video{camera_idx}")
    print(f"{'='*60}")

    try:
        local_ip = socket.gethostbyname(socket.gethostname())
    except:
        local_ip = '127.0.0.1'

    print(f"\n[Listen] VR の XRobot APP で:")
    print(f"  1. Camera ドロップダウンで 'ZEDMINI' を選択")
    print(f"  2. 'Listen' ボタンを押す")
    print(f"  3. IP に '{local_ip}' を入力")
    print(f"  4. Confirm を押す\n")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host if host != '0.0.0.0' else '', port))
    server.listen(1)
    print(f"[Listen] Waiting for connection on port {port}...")

    while True:
        try:
            conn, addr = server.accept()
            print(f"[Listen] Client connected from {addr}")

            # OPEN_CAMERA コマンドを受信
            data = b''
            config = None
            while len(data) < 1024:
                chunk = conn.recv(1024)
                if not chunk:
                    break
                data += chunk
                config = parse_open_camera(data)
                if config is not None:
                    break

            if config is None:
                print(f"[Listen] Failed to parse command ({len(data)} bytes)")
                conn.close()
                continue

            target_ip = config['ip']
            target_port = config['port']
            use_width = config['width'] if config['width'] > 0 else width
            use_height = config['height'] if config['height'] > 0 else height
            use_fps = config['fps'] if config['fps'] > 0 else fps
            use_bitrate = config['bitrate'] if config['bitrate'] > 0 else bitrate

            print(f"[Listen] Starting stream to {target_ip}:{target_port}")

            stream_webcam_pyav(
                target_ip, target_port, camera_idx,
                use_width, use_height, use_fps, use_bitrate
            )

            conn.close()
            print(f"\n[Listen] Stream ended. Waiting for new connection...")

        except KeyboardInterrupt:
            print("\n[Listen] Shutting down...")
            break
        except Exception as e:
            print(f"[Listen] Error: {e}")
            traceback.print_exc()

    server.close()


def send_mode(server_ip: str, server_port: int, camera_idx: int,
              width: int, height: int, fps: int, bitrate: int):
    """Direct send モード"""
    stream_webcam_pyav(server_ip, server_port, camera_idx,
                       width, height, fps, bitrate)


def main():
    parser = argparse.ArgumentParser(
        description="Webcam to VR Streamer (RobotVision-PC compatible)")
    parser.add_argument('--listen', type=str, default=None,
                        help='Listen address (e.g. 0.0.0.0:13579)')
    parser.add_argument('--send', action='store_true',
                        help='Direct send mode')
    parser.add_argument('--server', type=str, default='192.168.123.75',
                        help='VR headset IP (for --send mode)')
    parser.add_argument('--port', type=int, default=12345,
                        help='VR headset port (for --send mode)')
    parser.add_argument('--camera', type=int, default=0,
                        help='Camera device index (default: 0)')
    parser.add_argument('--width', type=int, default=1280,
                        help='Video width (default: 1280)')
    parser.add_argument('--height', type=int, default=720,
                        help='Video height (default: 720)')
    parser.add_argument('--fps', type=int, default=30,
                        help='Frame rate (default: 30)')
    parser.add_argument('--bitrate', type=int, default=4000000,
                        help='Bitrate in bps (default: 4000000)')
    args = parser.parse_args()

    if not HAS_PYAV:
        print("Error: PyAV is required. Install with: pip install av")
        sys.exit(1)

    if not args.listen and not args.send:
        print("Error: Either --listen or --send is required")
        parser.print_help()
        sys.exit(1)

    if args.listen:
        listen_mode(args.listen, args.camera, args.width, args.height,
                    args.fps, args.bitrate)
    elif args.send:
        send_mode(args.server, args.port, args.camera,
                  args.width, args.height, args.fps, args.bitrate)


if __name__ == '__main__':
    main()
