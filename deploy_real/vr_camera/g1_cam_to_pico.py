#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 頭カメラ (RealSense D435i RGB) → PICO (XRoboToolkit の XRobot アプリ「Remote Vision」) 映像送信サーバー

2026-02 に作った persistent_vr_server.py はリポジトリに入っておらず失われたため、2026-11 に作り直したもの。

プロトコル (deploy_real/data_utils/webcam_to_vr.py と同じ):
  1. このサーバーが TCP 13579 で待つ。
  2. PICO の XRobot アプリ: Remote Vision → カメラ(ZEDMINI / PICO4U)を選ぶ → Listen → G1 の IP → Confirm。
     アプリがここに接続して OPEN_CAMERA {width, height, fps, bitrate, hevc, port, ip} を送ってくる。
  3. こちらから ip:port に TCP 接続し、H.264 Annex-B を 1 フレーム 1 パケット、
     [4byte big-endian 長さ][データ] の形で送り続ける。
  幅が 2000 以上 (ZEDMINI=2560x720, PICO4U=2160x810 など) の要求は左右並びのステレオなので、
  モノラルの RGB を左右に並べて送る。

エンコードは Jetson のハードウェアエンコーダー (nvv4l2h264enc) を GStreamer で使うので CPU 負荷は小さい。

映像の元 (--source):
  v4l2      : (既定) /dev/video4 (D435i の RGB, YUYV) を直接開く。Unitree の videohub_pc4 を止めておく必要がある。
  multicast : Unitree の videohub_pc4 が出している H.264 RTP マルチキャスト (既定 230.1.1.1:1720) を
              受けてデコードし直す。2026-11-03 の G1 では何も流れておらず使えなかった。
  test      : テスト模様 (videotestsrc)。カメラなしで PICO までの経路を確かめる用。
  sim       : シミュレーターの G1 の頭カメラ目線。sim_head_cam.py (127.0.0.1:5601) が描いた絵を受け取る。

ホストの python3 (gi / GStreamer 1.16 + NVIDIA プラグイン) で動かす。conda の gmr 環境ではない。

例:
  # ネットワークなしの自己テスト (5 秒エンコードして /tmp に保存)
  python3 g1_cam_to_pico.py --source test --selftest 5 --size 2560x720
  # PICO 待ち受け
  python3 g1_cam_to_pico.py --source v4l2 --cam-size 960x540 --listen 192.168.123.164:13579
"""

import argparse
import fcntl
import glob
import os
import queue
import signal
import socket
import struct
import sys
import threading
import time
import traceback

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

try:
    gi.require_version("GstVideo", "1.0")
    from gi.repository import GstVideo  # noqa: E402
except (ValueError, ImportError):
    GstVideo = None

from status_overlay import StatusOverlay, has_japanese_font  # noqa: E402
from battery_dds import BatteryReader  # noqa: E402


# ---------------------------------------------------------------------------
# ログ
# ---------------------------------------------------------------------------
def log(msg):
    t = time.strftime("%H:%M:%S") + ".%03d" % int((time.time() % 1) * 1000)
    print("[%s] %s" % (t, msg), flush=True)


# ---------------------------------------------------------------------------
# PICO からのコマンドの解析
#   [4B BE body_len] body
#   body = [4B LE cmd_len][cmd][4B LE data_len][data]
#   OPEN_CAMERA の data = CA FE ver | w h fps bitrate hevc render_mode port (各 4B LE int)
#                         | 1B len + camera 名 | 1B len + ip
# ---------------------------------------------------------------------------
def split_command(buf):
    """buf の先頭から 1 コマンドを取り出す。足りなければ None。戻り値 (cmd, data, consumed)"""
    if len(buf) < 4:
        return None
    body_len = struct.unpack(">I", buf[:4])[0]
    if body_len > 1 << 20:
        raise ValueError("command too large: %d" % body_len)
    if len(buf) < 4 + body_len:
        return None
    body = buf[4:4 + body_len]
    if len(body) < 8:
        return ("", b"", 4 + body_len)
    cmd_len = struct.unpack("<I", body[:4])[0]
    cmd = body[4:4 + cmd_len].decode("utf-8", errors="replace").rstrip("\x00")
    off = 4 + cmd_len
    data = b""
    if len(body) >= off + 4:
        data_len = struct.unpack("<I", body[off:off + 4])[0]
        data = body[off + 4:off + 4 + data_len]
    return (cmd, data, 4 + body_len)


def parse_open_camera(data):
    if len(data) < 31 or data[0] != 0xCA or data[1] != 0xFE:
        raise ValueError("bad OPEN_CAMERA payload (%d bytes, head=%s)" % (len(data), data[:4].hex()))
    off = 3
    vals = struct.unpack("<7i", data[off:off + 28])
    off += 28
    width, height, fps, bitrate, hevc, render_mode, port = vals
    n = data[off]
    off += 1
    camera = data[off:off + n].decode("utf-8", errors="replace")
    off += n
    n = data[off] if off < len(data) else 0
    off += 1
    ip = data[off:off + n].decode("utf-8", errors="replace")
    return {
        "width": width, "height": height, "fps": fps, "bitrate": bitrate,
        "hevc": hevc, "render_mode": render_mode, "port": port,
        "camera": camera, "ip": ip, "version": data[2],
    }


# ---------------------------------------------------------------------------
# GStreamer パイプライン
# ---------------------------------------------------------------------------
def has_element(name):
    return Gst.ElementFactory.find(name) is not None


# --- v4l2 のデバイスを探す (USB がリセットされると /dev/video* の番号が変わるため) ---
_VIDIOC_QUERYCAP = 0x80685600            # _IOR('V', 0, struct v4l2_capability[104])
_VIDIOC_ENUM_FMT = 0xC0405602            # _IOWR('V', 2, struct v4l2_fmtdesc[64])
_VIDIOC_ENUM_FRAMESIZES = 0xC02C564A     # _IOWR('V', 74, struct v4l2_frmsizeenum[44])
_V4L2_CAP_VIDEO_CAPTURE = 0x00000001
_V4L2_CAP_DEVICE_CAPS = 0x80000000
_FOURCC_YUYV = 0x56595559


def _v4l2_info(path):
    """(カード名, YUYV で出せる大きさの集合) を返す。映像を取れないノードは None"""
    fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    try:
        cap = bytearray(104)
        fcntl.ioctl(fd, _VIDIOC_QUERYCAP, cap)
        card = bytes(cap[16:48]).split(b"\0")[0].decode("utf-8", "replace")
        caps, dcaps = struct.unpack_from("<II", cap, 84)
        if not ((dcaps if caps & _V4L2_CAP_DEVICE_CAPS else caps) & _V4L2_CAP_VIDEO_CAPTURE):
            return None
        fourccs = []
        for i in range(64):
            fd_buf = bytearray(64)
            struct.pack_into("<II", fd_buf, 0, i, 1)   # index, type=VIDEO_CAPTURE
            try:
                fcntl.ioctl(fd, _VIDIOC_ENUM_FMT, fd_buf)
            except OSError:
                break
            fourccs.append(struct.unpack_from("<I", fd_buf, 44)[0])
        if _FOURCC_YUYV not in fourccs:
            return card, set()
        sizes = set()
        for i in range(64):
            fs = bytearray(44)
            struct.pack_into("<II", fs, 0, i, _FOURCC_YUYV)
            try:
                fcntl.ioctl(fd, _VIDIOC_ENUM_FRAMESIZES, fs)
            except OSError:
                break
            if struct.unpack_from("<I", fs, 8)[0] == 1:   # DISCRETE
                sizes.add(struct.unpack_from("<II", fs, 12))
        return card, sizes
    finally:
        os.close(fd)


def find_v4l2_device(want, width, height, card_hint="RealSense"):
    """--device auto のとき、カード名に card_hint を含み YUYV で width x height を出せるノードを探す"""
    if want != "auto":
        return want
    found = []
    for path in sorted(glob.glob("/dev/video*"), key=lambda p: int("0" + "".join(c for c in p if c.isdigit()))):
        try:
            info = _v4l2_info(path)
        except OSError:
            continue
        if info and info[1]:
            found.append((path, info[0], info[1]))
    for path, card, sizes in found:
        if card_hint.lower() in card.lower() and (width, height) in sizes:
            return path
    for path, card, sizes in found:
        if card_hint.lower() in card.lower():
            log("注意: %s (%s) は %dx%d を出せないようです (出せる大きさ: %s)" % (
                path, card, width, height, sorted(sizes)))
            return path
    raise RuntimeError("YUYV を出せる %s のカメラが見つかりません (見つかったもの: %s)" % (
        card_hint, [(p, c) for p, c, _ in found] or "なし"))


def crop_box(src_w, src_h, dst_w, dst_h):
    """src を dst の縦横比に合わせて中央で切り取る範囲 (left, top, right, bottom)"""
    src_ar = src_w / float(src_h)
    dst_ar = dst_w / float(dst_h)
    # 848x480 (1.767) と 16:9 (1.778) のような 2% 未満の差は、切らずにわずかに引き伸ばす
    if abs(src_ar - dst_ar) / dst_ar < 0.02:
        return None
    if src_ar > dst_ar:  # 横が余る
        w = int(round(src_h * dst_ar)) // 2 * 2
        left = (src_w - w) // 2 // 2 * 2
        return (left, 0, left + w, src_h)
    h = int(round(src_w / dst_ar)) // 2 * 2
    top = (src_h - h) // 2 // 2 * 2
    return (0, top, src_w, top + h)


def build_pipeline_desc(args, out_w, out_h, bitrate, hevc):
    sbs = out_w >= args.sbs_min_width and not args.mono
    eye_w = out_w // 2 if sbs else out_w
    eye_h = out_h
    eye_w -= eye_w % 2
    eye_h -= eye_h % 2

    # --- 映像の元 ---
    if args.source == "v4l2":
        dev = find_v4l2_device(args.device, args.cam_width, args.cam_height, args.device_hint)
        if dev != args.device:
            log("カメラ: %s (自動で選択)" % dev)
        src = ("v4l2src device=%s do-timestamp=true ! "
               "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! "
               % (dev, args.cam_width, args.cam_height, args.cam_fps))
    elif args.source == "multicast":
        iface = ("multicast-iface=%s " % args.mcast_iface) if args.mcast_iface else ""
        dec = "nvv4l2decoder" if args.encoder == "nv" else "avdec_h264"
        src = ("udpsrc address=%s port=%d auto-multicast=true %s"
               "caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264\" ! "
               "rtpjitterbuffer latency=%d drop-on-latency=true ! rtph264depay ! h264parse ! %s ! "
               % (args.mcast_group, args.mcast_port, iface, args.jitter_ms, dec))
    elif args.source == "sim":
        # sim_head_cam.py から受け取る RGBA の生フレーム (SimFeeder が appsrc に押し込む)
        src = ("appsrc name=rawsrc is-live=true format=time do-timestamp=true block=false "
               "caps=video/x-raw,format=RGBA,width=%d,height=%d,framerate=%d/1 ! "
               % (args.cam_width, args.cam_height, args.cam_fps))
        if args.encoder != "nv":
            src += "videoconvert ! "
    else:  # test
        src = ("videotestsrc is-live=true pattern=%s ! "
               "video/x-raw,format=I420,width=%d,height=%d,framerate=%d/1 ! "
               % (args.test_pattern, args.cam_width, args.cam_height, args.cam_fps))
    src += "identity name=srcprobe silent=true ! "

    # --- 切り取り・縮小 (Jetson は nvvidconv = ハードウェア) ---
    crop = None
    if args.aspect == "crop" and args.source != "multicast":
        crop = crop_box(args.cam_width, args.cam_height, eye_w, eye_h)
    if args.encoder == "nv":
        conv = "nvvidconv name=scaler"   # 拡大縮小の方法 (interpolation-method) は作ったあとに設定する
        if crop:
            # 普通のメモリの絵を nvvidconv で直接切り取ると CPU が重くなった (11/4 の G1: 640x480 と 848x480 で
            # 1 コアの 55〜68%、切らない 960x540 は 25%)。先に GPU 側 (NVMM) に移してから切り取る。
            conv = ("nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
                    "nvvidconv name=scaler left=%d top=%d right=%d bottom=%d" % crop)
    else:
        conv = "videoconvert ! videoscale"
        if crop:
            l, t, r, b = crop
            conv = ("videocrop left=%d top=%d right=%d bottom=%d ! " % (
                l, t, args.cam_width - r, args.cam_height - b)) + conv

    if args.encoder == "nv":
        enc_in_caps = "video/x-raw(memory:NVMM),format=NV12"
    else:
        enc_in_caps = "video/x-raw,format=I420"

    # 腕テレオペの状態の文字 (片目の絵に描くので、左右並びでは両目の同じ位置に出る)
    ovl = ""
    if args.overlay_ok:
        size = args.overlay_font_size or max(12, int(round(eye_h * 0.042)))
        ovl = ('textoverlay name=ovl text="" auto-resize=false font-desc="%s %d" '
               'valignment=%s halignment=center ypad=%d shaded-background=true wait-text=false ! '
               % (args.overlay_font, size, args.overlay_pos, max(4, int(eye_h * 0.03))))
        # G1 のバッテリー残量を右上に小さく (2026-11-10〜)。PICO で端が切れないよう少し内側に置く
        if getattr(args, "battery_ok", False):
            bsize = args.battery_font_size or max(10, int(round(size * 0.65)))
            ovl += ('textoverlay name=bat text="" auto-resize=false font-desc="%s %d" '
                    'valignment=top halignment=right xpad=%d ypad=%d shaded-background=true wait-text=false ! '
                    % (args.overlay_font, bsize, max(4, int(eye_w * args.battery_xpad)),
                       max(4, int(eye_h * 0.03))))

    if sbs:
        # 左右に同じ映像を並べる (CPU の compositor はコピーだけなので軽い)
        scale = "%s ! video/x-raw,format=I420,width=%d,height=%d ! %s" % (conv, eye_w, eye_h, ovl)
        # Jetson ではエンコーダーの前に NVMM へ戻す (caps を 2 つ続けると parse_launch が失敗するので分ける)
        tail = ("nvvidconv ! %s,width=%d,height=%d ! " % (enc_in_caps, eye_w * 2, eye_h)
                if args.encoder == "nv" else "")
        # GStreamer 1.16 (Jetson) では compositor は plugins-bad。無ければ videomixer (plugins-good) を使う
        mixer = "compositor" if has_element("compositor") else "videomixer"
        body = (scale + "tee name=t "
                "t. ! queue max-size-buffers=1 leaky=downstream ! comp.sink_0 "
                "t. ! queue max-size-buffers=1 leaky=downstream ! comp.sink_1 "
                "%s name=comp background=black sink_0::xpos=0 sink_1::xpos=%d ! "
                "video/x-raw,format=I420,width=%d,height=%d ! %s"
                % (mixer, eye_w, eye_w * 2, eye_h, tail))
    elif ovl:
        # 文字を描くために一度 CPU の I420 にして、エンコーダーの前で戻す
        back = ("nvvidconv ! %s,width=%d,height=%d ! " % (enc_in_caps, eye_w, eye_h)
                if args.encoder == "nv" else "")
        body = "%s ! video/x-raw,format=I420,width=%d,height=%d ! %s%s" % (conv, eye_w, eye_h, ovl, back)
    else:
        body = "%s ! %s,width=%d,height=%d ! " % (conv, enc_in_caps, eye_w, eye_h)

    # --- エンコード ---
    gop = max(1, int(args.gop))
    if args.encoder == "nv":
        if hevc:
            enc = ("nvv4l2h265enc bitrate=%d control-rate=1 iframeinterval=%d idrinterval=%d "
                   "insert-sps-pps=true insert-vui=true maxperf-enable=true preset-level=1 ! "
                   "h265parse config-interval=-1 ! video/x-h265,stream-format=byte-stream,alignment=au ! "
                   % (bitrate, gop, gop))
        else:
            enc = ("nvv4l2h264enc bitrate=%d control-rate=1 profile=0 iframeinterval=%d idrinterval=%d "
                   "insert-sps-pps=true insert-vui=true maxperf-enable=true preset-level=1 ! "
                   "h264parse config-interval=-1 ! video/x-h264,stream-format=byte-stream,alignment=au ! "
                   % (bitrate, gop, gop))
    else:  # x264 (Jetson 以外でのテスト用)
        if hevc:
            enc = ("x265enc bitrate=%d speed-preset=ultrafast tune=zerolatency key-int-max=%d ! "
                   "h265parse config-interval=-1 ! video/x-h265,stream-format=byte-stream,alignment=au ! "
                   % (bitrate // 1000, gop))
        else:
            enc = ("x264enc bitrate=%d speed-preset=ultrafast tune=zerolatency key-int-max=%d "
                   "bframes=0 byte-stream=true ! video/x-h264,profile=baseline ! "
                   "h264parse config-interval=-1 ! video/x-h264,stream-format=byte-stream,alignment=au ! "
                   % (bitrate // 1000, gop))

    sink = "appsink name=sink emit-signals=true sync=false max-buffers=4 drop=false"
    return src + body + enc + sink, sbs, eye_w, eye_h


class SimFeeder(object):
    """sim_head_cam.py (127.0.0.1:5601) から RGBA フレームを受け取り appsrc に渡す"""
    MAGIC = b"G1RAW1"

    def __init__(self, addr):
        host, port = addr.rsplit(":", 1)
        try:
            self.sock = socket.create_connection((host, int(port)), timeout=3)
        except OSError as e:
            raise RuntimeError("シムの頭カメラ描画 (sim_head_cam.py, %s) につながりません: %s" % (addr, e))
        self.sock.settimeout(3)
        hdr = self._recv_exact(len(self.MAGIC) + 16)
        if hdr[:len(self.MAGIC)] != self.MAGIC:
            raise RuntimeError("sim_head_cam.py からの返事が想定と違います: %r" % hdr[:8])
        self.width, self.height, self.fps, fmt = struct.unpack("<IIII", hdr[len(self.MAGIC):])
        self.frame_size = self.width * self.height * 4
        self.stop_event = threading.Event()
        self.thread = None
        self.appsrc = None
        self.pushed = 0
        self.skipped = 0
        log("シム描画につながりました %dx%d %dfps" % (self.width, self.height, self.fps))

    def _recv_exact(self, n, buf=None):
        if buf is None:
            buf = bytearray(n)
        view = memoryview(buf)
        got = 0
        while got < n:
            k = self.sock.recv_into(view[got:], n - got)
            if k == 0:
                raise ConnectionError("sim_head_cam.py との接続が切れました")
            got += k
        return buf

    def start(self, appsrc):
        self.appsrc = appsrc
        self.thread = threading.Thread(target=self._run, name="simfeed", daemon=True)
        self.thread.start()

    def _run(self):
        buf = bytearray(self.frame_size)
        try:
            while not self.stop_event.is_set():
                self._recv_exact(self.frame_size, buf)
                # 下流が詰まっているときは古い絵を溜めずに捨てる (遅れを増やさない)
                if self.appsrc.get_property("current-level-bytes") > 2 * self.frame_size:
                    self.skipped += 1
                    continue
                self.appsrc.emit("push-buffer", Gst.Buffer.new_wrapped(bytes(buf)))
                self.pushed += 1
        except Exception as e:
            if not self.stop_event.is_set():
                log("シム描画からの受信が止まりました: %s" % e)
        finally:
            try:
                self.appsrc.emit("end-of-stream")
            except Exception:
                pass

    def stop(self):
        self.stop_event.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
        if self.thread:
            self.thread.join(timeout=3)


class Encoder(object):
    """パイプラインを動かし、エンコード済みフレームを on_frame(bytes, is_key) に渡す"""

    def __init__(self, args, out_w, out_h, bitrate, hevc, on_frame):
        self.args = args
        self.on_frame = on_frame
        self.feeder = None
        if args.source == "sim":
            self.feeder = SimFeeder(args.sim_addr)
            args.cam_width, args.cam_height, args.cam_fps = (
                self.feeder.width, self.feeder.height, self.feeder.fps)
        desc, sbs, eye_w, eye_h = build_pipeline_desc(args, out_w, out_h, bitrate, hevc)
        try:
            self.pipeline = Gst.parse_launch(desc)
        except Exception as e:
            if not args.overlay_ok:
                raise
            # 状態の文字のせいで映像が止まらないよう、重ねずに作り直す
            log("注意: 状態の重ね表示を入れるとパイプラインを作れません (%s)。重ねずに続けます" % e)
            args.overlay_ok = False
            desc, sbs, eye_w, eye_h = build_pipeline_desc(args, out_w, out_h, bitrate, hevc)
            self.pipeline = Gst.parse_launch(desc)
        log("pipeline: %s" % desc)
        log("出力 %dx%d (%s, 片目 %dx%d), %d kbps, %s" % (
            eye_w * (2 if sbs else 1), eye_h, "左右ステレオ" if sbs else "モノラル",
            eye_w, eye_h, bitrate // 1000, "H.265" if hevc else "H.264"))
        self.sink = self.pipeline.get_by_name("sink")
        self.sink.connect("new-sample", self._on_sample)
        self.enc_elem = None
        it = self.pipeline.iterate_elements()
        while True:
            ok, el = it.next()
            if ok != Gst.IteratorResult.OK:
                break
            if el.get_factory() and "enc" in el.get_factory().get_name():
                self.enc_elem = el
        self.frames = 0
        self.bytes = 0
        self.src_caps_logged = False
        # nvvidconv の拡大縮小は既定が Nearest (ブロック状に粗くなる) なので変える。無い版なら既定のまま
        scaler = self.pipeline.get_by_name("scaler")
        if scaler is not None and args.interp >= 0:
            try:
                scaler.set_property("interpolation-method", args.interp)
            except Exception as e:
                log("注意: nvvidconv の interpolation-method を %d にできません (%s)。既定のまま" % (args.interp, e))
        self.overlay = None
        el = self.pipeline.get_by_name("ovl")
        if el is not None:
            self.overlay = StatusOverlay(el, args, log, battery_el=self.pipeline.get_by_name("bat"),
                                         battery=getattr(args, "battery", None))

    def _on_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.EOS
        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR
        try:
            data = bytes(info.data)
        finally:
            buf.unmap(info)
        is_key = not buf.has_flags(Gst.BufferFlags.DELTA_UNIT)
        self.frames += 1
        self.bytes += len(data)
        try:
            self.on_frame(data, is_key)
        except Exception as e:  # 送信側の問題でパイプラインを止めない
            log("on_frame error: %s" % e)
        return Gst.FlowReturn.OK

    def request_keyframe(self):
        if GstVideo is None or self.enc_elem is None:
            return
        try:
            ev = GstVideo.video_event_new_upstream_force_key_unit(Gst.CLOCK_TIME_NONE, True, 0)
            self.enc_elem.get_static_pad("src").send_event(ev)
        except Exception as e:
            log("keyframe request failed: %s" % e)

    def start(self):
        r = self.pipeline.set_state(Gst.State.PLAYING)
        if r == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("パイプラインを開始できません (カメラが使用中か、要素が足りない可能性)")
        if self.feeder:
            self.feeder.start(self.pipeline.get_by_name("rawsrc"))
        if self.overlay:
            self.overlay.start()

    def poll_bus(self, timeout_ms=100):
        """エラー/EOS なら文字列を返す"""
        bus = self.pipeline.get_bus()
        msg = bus.timed_pop_filtered(
            timeout_ms * Gst.MSECOND,
            Gst.MessageType.ERROR | Gst.MessageType.EOS | Gst.MessageType.WARNING)
        if not self.src_caps_logged:
            el = self.pipeline.get_by_name("srcprobe")
            caps = el.get_static_pad("src").get_current_caps() if el else None
            if caps:
                log("映像の元: %s" % caps.to_string())
                self.src_caps_logged = True
        if msg is None:
            return None
        if msg.type == Gst.MessageType.WARNING:
            w, dbg = msg.parse_warning()
            log("GStreamer warning: %s (%s)" % (w, dbg))
            return None
        if msg.type == Gst.MessageType.ERROR:
            e, dbg = msg.parse_error()
            return "error: %s (%s)" % (e, dbg)
        return "eos"

    def stop(self):
        if self.overlay:
            self.overlay.stop()
        if self.feeder:
            self.feeder.stop()
        try:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline.get_state(3 * Gst.SECOND)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# PICO への送信
# ---------------------------------------------------------------------------
class Stream(object):
    """1 回分の配信 (OPEN_CAMERA 1 つに対応)。別スレッドで動く"""

    def __init__(self, args, config):
        self.args = args
        self.cfg = config
        self.stop_event = threading.Event()
        self.q = queue.Queue(maxsize=args.max_queue)
        self.wait_key = False
        self.dropped = 0
        self.sent = 0
        self.sent_bytes = 0
        self.thread = threading.Thread(target=self._run, name="stream", daemon=True)

    def start(self):
        self.thread.start()

    def stop(self, why=""):
        if not self.stop_event.is_set():
            log("配信を止めます %s" % why)
        self.stop_event.set()
        self.thread.join(timeout=5)

    def alive(self):
        return self.thread.is_alive()

    def _on_frame(self, data, is_key):
        if self.wait_key:
            if not is_key:
                self.dropped += 1
                return
            self.wait_key = False
        try:
            self.q.put_nowait((data, is_key))
        except queue.Full:
            # 送信が詰まっている: 古いフレームを全部捨てて次のキーフレームから再開
            n = 0
            while True:
                try:
                    self.q.get_nowait()
                    n += 1
                except queue.Empty:
                    break
            self.dropped += n + 1
            self.wait_key = True
            if self.enc:
                self.enc.request_keyframe()

    def _connect(self, ip, port):
        for i in range(self.args.connect_retries):
            if self.stop_event.is_set():
                return None
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.settimeout(3)
                s.connect((ip, port))
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(self.args.send_timeout)
                log("PICO %s:%d に接続しました" % (ip, port))
                return s
            except Exception as e:
                s.close()
                log("PICO %s:%d に接続できません (%d/%d): %s" % (
                    ip, port, i + 1, self.args.connect_retries, e))
                time.sleep(1)
        return None

    def _run(self):
        cfg = self.cfg
        self.enc = None
        sock = None
        try:
            w = cfg["width"] if cfg["width"] > 0 else self.args.width
            h = cfg["height"] if cfg["height"] > 0 else self.args.height
            if self.args.force_size:
                w, h = self.args.width, self.args.height
            br = self.args.bitrate or (cfg["bitrate"] if cfg["bitrate"] > 0 else 4000000)
            hevc = bool(cfg["hevc"]) and not self.args.no_hevc
            ip = self.args.pico_ip or cfg["ip"]
            port = cfg["port"]
            if not ip or port <= 0:
                log("送り先が分かりません ip=%r port=%r" % (ip, port))
                return
            # カメラ・エンコーダーを先に準備してから接続 (webcam_to_vr.py と同じ順)
            self.enc = Encoder(self.args, w, h, br, hevc, self._on_frame)
            sock = self._connect(ip, port)
            if sock is None:
                return
            self.enc.start()
            t0 = time.time()
            last = t0
            last_sent = 0
            last_bytes = 0
            cpu0 = os.times()
            warned_nosrc = False
            while not self.stop_event.is_set():
                err = self.enc.poll_bus(0)
                if err:
                    log("パイプライン停止: %s" % err)
                    return
                try:
                    data, is_key = self.q.get(timeout=0.2)
                except queue.Empty:
                    data = None
                if data is not None:
                    try:
                        sock.sendall(struct.pack(">I", len(data)) + data)
                    except (OSError, socket.timeout) as e:
                        log("PICO との接続が切れました: %s" % e)
                        return
                    if self.sent < 3:
                        log("frame %d: %d bytes key=%s head=%s" % (
                            self.sent, len(data), is_key, data[:8].hex()))
                    self.sent += 1
                    self.sent_bytes += len(data)
                now = time.time()
                if not warned_nosrc and not self.enc.src_caps_logged and now - t0 > 3:
                    log("注意: 映像の元から 3 秒たっても 1 枚も届いていません (黒い画面になります)")
                    warned_nosrc = True
                if now - last >= self.args.stats_sec:
                    cpu1 = os.times()
                    dt = now - last
                    cpu = 100.0 * ((cpu1.user - cpu0.user) + (cpu1.system - cpu0.system)) / dt
                    ov = self.enc.overlay
                    log("送信 %.1f fps, %.0f kbps, 捨て %d, 待ち %d, CPU %.0f%% (経過 %.0f 秒)%s" % (
                        (self.sent - last_sent) / dt, (self.sent_bytes - last_bytes) * 8 / dt / 1000,
                        self.dropped, self.q.qsize(), cpu, now - t0,
                        ((", 表示 [%s] %s" % (ov.level, ov.text)) if ov else "")
                        + ((", %s" % ov.bat_text) if ov and ov.bat_text else "")))
                    last, last_sent, last_bytes, cpu0 = now, self.sent, self.sent_bytes, cpu1
        except Exception as e:
            log("配信エラー: %s" % e)
            traceback.print_exc()
        finally:
            if self.enc:
                self.enc.stop()
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
            log("配信終了 (送信 %d フレーム, %.1f MB, 捨て %d)" % (
                self.sent, self.sent_bytes / 1e6, self.dropped))


# ---------------------------------------------------------------------------
# 待ち受け
# ---------------------------------------------------------------------------
class Server(object):
    def __init__(self, args):
        self.args = args
        self.current = None
        self.lock = threading.Lock()
        self.stopping = threading.Event()

    def start_stream(self, cfg):
        with self.lock:
            if self.current:
                self.current.stop("(新しい OPEN_CAMERA が来たため)")
            self.current = Stream(self.args, cfg)
            self.current.start()

    def stop_stream(self, why):
        with self.lock:
            if self.current:
                self.current.stop(why)
                self.current = None

    def handle_control(self, conn, addr):
        """PICO からの制御接続。コマンドを読み続ける"""
        buf = b""
        conn.settimeout(1.0)
        try:
            while not self.stopping.is_set():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    log("制御接続が閉じられました %s (配信は続けます)" % (addr,))
                    return
                buf += chunk
                while True:
                    r = split_command(buf)
                    if r is None:
                        break
                    cmd, data, used = r
                    buf = buf[used:]
                    log("コマンド受信 %r (%d bytes) from %s" % (cmd, len(data), addr[0]))
                    if cmd == "OPEN_CAMERA":
                        try:
                            cfg = parse_open_camera(data)
                        except Exception as e:
                            log("OPEN_CAMERA を読めません: %s" % e)
                            continue
                        log("要求: %s" % cfg)
                        self.start_stream(cfg)
                    elif cmd in ("CLOSE_CAMERA", "STOP_CAMERA"):
                        self.stop_stream("(PICO から %s)" % cmd)
        except Exception as e:
            log("制御接続エラー: %s" % e)
        finally:
            try:
                conn.close()
            except Exception:
                pass

    def serve(self):
        host, port = self.args.listen.rsplit(":", 1)
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, int(port)))
        srv.listen(4)
        srv.settimeout(1.0)
        log("待ち受け %s:%s 映像の元=%s エンコーダー=%s" % (host, port, self.args.source, self.args.encoder))
        log("PICO: XRobot アプリ → Remote Vision → カメラを選ぶ → Listen → このPCのIP → Confirm")
        try:
            while not self.stopping.is_set():
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    with self.lock:
                        if self.current and not self.current.alive():
                            self.current = None
                            log("配信が終わりました。PICO で Listen し直すと再開します")
                    continue
                log("PICO から接続 %s:%d" % addr)
                threading.Thread(target=self.handle_control, args=(conn, addr), daemon=True).start()
        finally:
            srv.close()
            self.stop_stream("(サーバー終了)")


# ---------------------------------------------------------------------------
# 自己テスト (ネットワークなし)
# ---------------------------------------------------------------------------
def selftest(args):
    w, h = args.width, args.height
    out = args.selftest_out
    f = open(out, "wb")
    stats = {"n": 0, "key": 0, "bytes": 0, "first": None}

    def on_frame(data, is_key):
        f.write(data)
        stats["n"] += 1
        stats["bytes"] += len(data)
        stats["key"] += int(is_key)
        if stats["first"] is None:
            stats["first"] = data[:64]

    enc = Encoder(args, w, h, args.bitrate or 4000000, False, on_frame)
    cpu0 = os.times()
    t0 = time.time()
    enc.start()
    err = None
    while time.time() - t0 < args.selftest:
        err = enc.poll_bus(100)
        if err:
            break
    enc.stop()
    f.close()
    dt = time.time() - t0
    cpu1 = os.times()
    cpu = 100.0 * ((cpu1.user - cpu0.user) + (cpu1.system - cpu0.system)) / dt
    first = stats["first"] or b""
    nal_types = []
    i = 0
    while i < len(first) - 4 and len(nal_types) < 4:
        if first[i:i + 4] == b"\x00\x00\x00\x01":
            nal_types.append(first[i + 4] & 0x1F)
            i += 4
        elif first[i:i + 3] == b"\x00\x00\x01":
            nal_types.append(first[i + 3] & 0x1F)
            i += 3
        else:
            i += 1
    if err:
        result = "NG (%s)" % err
    elif stats["n"] == 0 or not enc.src_caps_logged:
        # 左右並びの合成 (compositor) は入力が無くても黒い画面を出すことがあるので、映像の元を直接見る
        result = "NG (映像の元から 1 枚も届いていません)"
    else:
        result = "OK"
    log("自己テスト結果: %s" % result)
    log("  %.1f 秒, %d フレーム (%.1f fps), キーフレーム %d, %.0f kbps, CPU %.0f%%" % (
        dt, stats["n"], stats["n"] / dt, stats["key"], stats["bytes"] * 8 / dt / 1000, cpu))
    log("  先頭フレームの NAL 種類 %s (7=SPS 8=PPS 5=IDR が並べば PICO で再生できる形)" % nal_types)
    log("  保存先 %s (ffprobe や ffplay で確認可)" % out)
    return 0 if (not err and stats["n"] > 0) else 1


# ---------------------------------------------------------------------------
def parse_size(s):
    w, h = s.lower().split("x")
    return int(w), int(h)


def main():
    p = argparse.ArgumentParser(description="G1 カメラ → PICO XRobot アプリ 映像送信")
    p.add_argument("--listen", default="0.0.0.0:13579", help="待ち受け (既定 0.0.0.0:13579)")
    p.add_argument("--source", choices=["v4l2", "multicast", "test", "sim"], default="v4l2")
    p.add_argument("--sim-addr", default="127.0.0.1:5601", help="sim_head_cam.py の待ち受け (--source sim)")
    p.add_argument("--device", default="auto",
                   help="v4l2 のデバイス。auto = 配信のたびに RealSense の RGB (YUYV) のノードを探す "
                        "(USB リセットで /dev/video4 の番号が変わることがあるため)")
    p.add_argument("--device-hint", default="RealSense", help="--device auto で探すカメラの名前の一部")
    p.add_argument("--cam-size", default="960x540",
                   help="カメラ側の解像度 (v4l2/test)。11/4 の G1 は 1280x720 が不安定だったので 960x540 (16:9 で全体が入る)")
    p.add_argument("--cam-fps", type=int, default=30)
    p.add_argument("--mcast-group", default="230.1.1.1")
    p.add_argument("--mcast-port", type=int, default=1720)
    p.add_argument("--mcast-iface", default="", help="例 eth0 (空なら OS 任せ)")
    p.add_argument("--jitter-ms", type=int, default=20)
    p.add_argument("--test-pattern", default="ball")
    p.add_argument("--encoder", choices=["nv", "x264"], default="nv",
                   help="nv=Jetson ハードウェア, x264=ソフト (Jetson 以外でのテスト用)")
    p.add_argument("--size", default="2560x720",
                   help="PICO が大きさを指定しないとき・自己テストの出力サイズ")
    p.add_argument("--force-size", action="store_true", help="PICO の要求サイズを無視して --size を使う")
    p.add_argument("--mono", action="store_true", help="幅が大きくても左右に並べない")
    p.add_argument("--sbs-min-width", type=int, default=2000, help="この幅以上なら左右並びにする")
    p.add_argument("--aspect", choices=["crop", "stretch"], default="crop",
                   help="片目の縦横比に合わせて中央を切り取る(crop) か引き伸ばす(stretch)")
    p.add_argument("--interp", type=int, default=1,
                   help="nvvidconv の拡大縮小の方法 0=Nearest 1=Bilinear 2=5-Tap 3=10-Tap 4=Smart 5=Nicest (-1 で既定のまま)")
    p.add_argument("--bitrate", type=int, default=0, help="bps。0 なら PICO の要求値")
    p.add_argument("--no-hevc", action="store_true", help="PICO が H.265 を要求しても H.264 で送る")
    p.add_argument("--gop", type=int, default=30, help="キーフレーム間隔 (フレーム数)")
    p.add_argument("--pico-ip", default="", help="PICO の IP を固定する (空なら要求に書かれた IP)")
    p.add_argument("--max-queue", type=int, default=6, help="送信待ちがこれを超えたら捨てて次のキーフレームから")
    p.add_argument("--send-timeout", type=float, default=2.0)
    p.add_argument("--connect-retries", type=int, default=10)
    p.add_argument("--stats-sec", type=float, default=5.0)
    p.add_argument("--selftest", type=float, default=0, help="秒数。ネットワークなしでエンコードだけ試す")
    p.add_argument("--selftest-out", default="/tmp/g1_cam_selftest.h264")
    # 腕テレオペの状態の重ね表示 (Redis teleop_display を読むだけ)
    p.add_argument("--no-overlay", action="store_true", help="腕テレオペの状態を映像に重ねない")
    p.add_argument("--overlay-key", default="teleop_display", help="状態の Redis キー (IK が書く JSON)")
    p.add_argument("--overlay-lang", choices=["auto", "ja", "en"], default="auto",
                   help="auto = 日本語フォントがあれば ja、無ければ en")
    p.add_argument("--overlay-pos", choices=["top", "bottom"], default="top", help="文字の位置")
    p.add_argument("--overlay-font", default="Sans Bold", help="pango のフォント名")
    p.add_argument("--overlay-font-size", type=int, default=0, help="0 なら片目の高さに合わせる (720 で 30)")
    p.add_argument("--overlay-stale", type=float, default=0.5, help="状態がこの秒数より古ければ IK停止 と出す")
    p.add_argument("--overlay-hz", type=float, default=10.0, help="状態を読む回数 (1 秒あたり)")
    p.add_argument("--redis-host", default="127.0.0.1")
    p.add_argument("--redis-port", type=int, default=6379)
    # G1 のバッテリー残量 (DDS rt/lf/bmsstate を読むだけ) を右上に小さく出す
    p.add_argument("--no-battery", action="store_true", help="バッテリー残量を出さない")
    p.add_argument("--battery-net", default="eth0", help="DDS を使うネットワーク (ブリッジと同じ eth0)")
    p.add_argument("--battery-topic", default="rt/lf/bmsstate")
    p.add_argument("--battery-font-size", type=int, default=0, help="0 なら状態の文字の 0.65 倍")
    p.add_argument("--battery-xpad", type=float, default=0.10,
                   help="右端からの距離 (片目の幅に対する割合)。PICO で端が切れるなら大きく")
    p.add_argument("--battery-fake", type=int, default=None, help="テスト用: DDS を使わずこの値 [%%] を出す")
    args = p.parse_args()
    args.cam_width, args.cam_height = parse_size(args.cam_size)
    args.width, args.height = parse_size(args.size)

    Gst.init(None)
    need = ["appsink", "h264parse", "tee"]
    if not (has_element("compositor") or has_element("videomixer")):
        need.append("compositor")
    need += ["nvvidconv", "nvv4l2h264enc"] if args.encoder == "nv" else ["x264enc", "videoconvert"]
    if args.source == "multicast":
        need += ["udpsrc", "rtph264depay", "nvv4l2decoder" if args.encoder == "nv" else "avdec_h264"]
    elif args.source == "v4l2":
        need += ["v4l2src"]
    elif args.source == "sim":
        need += ["appsrc"]
    missing = [n for n in need if not has_element(n)]
    if missing:
        log("GStreamer の要素が足りません: %s" % missing)
        return 2

    # 状態の重ね表示は無くても映像は送れるので、足りなければ表示なしで続ける
    args.overlay_ok = False
    if not args.no_overlay:
        if not has_element("textoverlay"):
            log("注意: textoverlay (GStreamer の pango) が無いので、腕テレオペの状態は重ねません")
        else:
            args.overlay_ok = True
            if args.overlay_lang == "auto":
                fam = has_japanese_font()
                args.overlay_lang = "ja" if fam else "en"
                log("状態表示: %s" % ("日本語 (フォント %s)" % fam if fam else
                                     "英語 (日本語フォントが見つからないため)"))

    # バッテリー残量: 読めなければ出さないだけで、映像は続ける
    args.battery, args.battery_ok = None, False
    if args.overlay_ok and not args.no_battery:
        args.battery = BatteryReader(args.battery_net, args.battery_topic, log, fake=args.battery_fake)
        args.battery_ok = args.battery.start()

    if args.selftest > 0:
        return selftest(args)

    server = Server(args)

    def on_signal(signum, frame):
        log("終了シグナル %d" % signum)
        server.stopping.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    server.serve()
    log("終了")
    return 0


if __name__ == "__main__":
    sys.exit(main())
