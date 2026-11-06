#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
pano_web の送信側を GStreamer で行う（Ubuntu 用）。

Chrome の送信ページ（sender.html）の代わりに、これが PICO（viewer.html）からの接続要求に答える。
圧縮はハード（NVIDIA の NVENC、Intel/AMD の VA など）を自動で選び、無ければソフト（x264）にする。

  Insta360 X5 --USB--> gst_sender.py (GStreamer, ハード圧縮) ==WebRTC==> PICO のブラウザ (viewer.html)
                           \__ pano_server.py: ページ配信とつなぐための情報の受け渡し（今までと同じ）

使い方（pano_server.py を先に動かしておく。start_gst.sh なら両方まとめて起動する）:
  python3 gst_sender.py --list                 # カメラと、出せる形式・解像度の一覧
  python3 gst_sender.py                        # Insta360 を探して 2880x1440 30fps で送る
  python3 gst_sender.py --test                 # カメラなしの試験映像で送る
  python3 gst_sender.py --encoder x264enc      # 圧縮を指定（比べたいとき）

sender.html（Chrome の送信ページ）とは同時に使わない（どちらが答えるか分からなくなる）。
必要なもの（Ubuntu）: sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-{base,good,bad,ugly} \
    gstreamer1.0-nice gstreamer1.0-libav gir1.2-gst-plugins-bad-1.0 python3-gi python3-gst-1.0 v4l-utils
"""

import argparse
import datetime
import json
import os
import re
import sys
import threading
import time
import urllib.error
import urllib.request

try:
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstWebRTC", "1.0")
    gi.require_version("GstSdp", "1.0")
    from gi.repository import GLib, Gst, GstSdp, GstWebRTC  # noqa: E402
except (ImportError, ValueError) as e:
    sys.stderr.write(
        "GStreamer の Python 部品が見つかりません (%s)。\n"
        "conda の python ではなく /usr/bin/python3 で動かすか、次を入れてください:\n"
        "  sudo apt install python3-gi python3-gst-1.0 gir1.2-gst-plugins-bad-1.0 gstreamer1.0-nice\n" % e)
    sys.exit(2)

HERE = os.path.dirname(os.path.abspath(__file__))

# 圧縮の候補（上から順に、この PC で使えるものを選ぶ）
ENCODERS = [
    "nvh264enc",          # NVIDIA NVENC（GStreamer 1.20〜1.24 の名前）
    "nvcudah264enc",      # NVIDIA NVENC（新しい名前）
    "nvautogpuh264enc",
    "vah264lpenc",        # Intel / AMD（VA、省電力の回路）
    "vah264enc",
    "vaapih264enc",       # Intel / AMD（古い VA-API）
    "qsvh264enc",         # Intel Quick Sync
    "x264enc",            # ソフト
    "openh264enc",        # ソフト（Chrome の Windows 版と同じもの）
]
HARDWARE = {"nvh264enc", "nvcudah264enc", "nvautogpuh264enc", "vah264lpenc", "vah264enc", "vaapih264enc", "qsvh264enc"}

_log_file = None
_server = None


def log(msg, to_server=True):
    line = "[%s] %s" % (datetime.datetime.now().strftime("%H:%M:%S"), msg)
    print(line, flush=True)
    if _log_file:
        _log_file.write(line + "\n")
        _log_file.flush()
    if to_server and _server:
        threading.Thread(target=_post_quiet, args=("/api/log", {"role": "gst", "text": msg}), daemon=True).start()


def _post_quiet(path, obj):
    try:
        http_json("POST", path, obj, timeout=3)
    except Exception:  # noqa: BLE001
        pass


def http_json(method, path, obj=None, timeout=15):
    data = json.dumps(obj).encode("utf-8") if obj is not None else None
    req = urllib.request.Request(_server + path, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            body = r.read()
            return r.status, (json.loads(body.decode("utf-8")) if body else None)
    except urllib.error.HTTPError as e:
        body = e.read()
        try:
            return e.code, json.loads(body.decode("utf-8")) if body else None
        except ValueError:
            return e.code, None


# ---------------------------------------------------------------- camera
def list_cameras():
    """v4l2 のカメラを [(名前, /dev/videoN, Gst.Caps)] で返す"""
    mon = Gst.DeviceMonitor.new()
    mon.add_filter("Video/Source", None)
    mon.start()
    out = []
    try:
        for dev in mon.get_devices():
            props = dev.get_properties()
            api = props.get_string("device.api") if props else None
            if api and api != "v4l2":
                continue  # pipewire / libcamera の重複は除く
            path = None
            if props:
                path = props.get_string("api.v4l2.path") or props.get_string("device.path")
            if not path:
                try:
                    el = dev.create_element(None)
                    if el.get_factory().get_name() != "v4l2src":
                        continue
                    path = el.get_property("device")
                except Exception:  # noqa: BLE001
                    continue
            out.append((dev.get_display_name(), path, dev.get_caps()))
    finally:
        mon.stop()
    out.sort(key=lambda d: d[1])
    return out


def caps_modes(caps):
    """Gst.Caps → [(形式, 幅, 高さ, [fps...])]。形式は raw/YUY2 のように"""
    modes = []
    if caps is None:
        return modes
    for i in range(caps.get_size()):
        feat = caps.get_features(i)
        if feat is not None and "memory:" in feat.to_string() and "SystemMemory" not in feat.to_string():
            continue
        s = caps.get_structure(i)
        name = s.get_name()
        if name == "video/x-raw":
            fmt = "raw/" + (s.get_string("format") or "?")
        elif name == "image/jpeg":
            fmt = "mjpeg"
        elif name == "video/x-h264":
            fmt = "h264"
        elif name == "video/x-h265":
            fmt = "h265"
        else:
            fmt = name
        ok_w, w = s.get_int("width")
        ok_h, h = s.get_int("height")
        if not (ok_w and ok_h):
            continue
        modes.append((fmt, w, h, structure_fps(s)))
    return modes


def structure_fps(s):
    """framerate フィールドから fps の候補を取り出す（分数・リスト・範囲に対応）"""
    text = s.to_string()
    m = re.search(r"framerate=\(fraction\)\{?([^;}]*)\}?", text)
    if not m:
        m = re.search(r"framerate=\{?\s*\(fraction\)([^;}]*)\}?", text)
    fps = []
    if m:
        for part in re.findall(r"(\d+)/(\d+)", m.group(1)):
            n, d = int(part[0]), int(part[1])
            if d:
                fps.append(round(n / d, 2))
    return sorted(set(fps), reverse=True)


def pick_mode(caps, width, height, fps, want_fmt):
    """希望に一番近いモードを選ぶ。戻り値 (形式, 幅, 高さ, fps)"""
    modes = caps_modes(caps)
    if not modes:
        return None
    order = {"raw": 0, "mjpeg": 1, "h264": 2}

    def fmt_rank(f):
        base = f.split("/")[0]
        if want_fmt != "auto":
            return 0 if base == want_fmt else 9
        return order.get(base, 5)

    def score(m):
        f, w, h, rates = m
        best_fps = max(rates) if rates else 0
        size_pen = 0 if (w, h) == (width, height) else 1 + abs(w * h - width * height) / float(width * height)
        aspect_pen = 0 if abs(w / float(h) - 2.0) < 0.02 else 3
        fps_pen = 0 if best_fps + 0.5 >= fps else 2 + (fps - best_fps) / max(fps, 1)
        return (aspect_pen + size_pen + fps_pen, fmt_rank(f))

    f, w, h, rates = min(modes, key=score)
    if not rates or any(abs(r - fps) < 0.5 for r in rates):
        use_fps = next((r for r in rates if abs(r - fps) < 0.5), fps)
    else:
        lower = [r for r in rates if r <= fps]
        use_fps = max(lower) if lower else min(rates)
    return f, w, h, use_fps


def fps_fraction(fps):
    if abs(fps - round(fps)) < 0.01:
        return "%d/1" % round(fps)
    if abs(fps - 29.97) < 0.01:
        return "30000/1001"
    return "%d/1000" % round(fps * 1000)


# ---------------------------------------------------------------- encoder
def has(name):
    return Gst.ElementFactory.find(name) is not None


def pick_encoder(name):
    if name != "auto":
        if not has(name):
            raise SystemExit("圧縮 %s はこの PC の GStreamer にありません。--list で使えるものを見てください" % name)
        return name
    for n in ENCODERS:
        if has(n) and _encoder_opens(n):
            return n
    raise SystemExit("H.264 の圧縮が一つも見つかりません。gstreamer1.0-plugins-ugly（x264）を入れてください")


def _encoder_opens(name):
    """要素があっても GPU が使えないと開けないことがあるので、READY まで試す"""
    try:
        el = Gst.ElementFactory.make(name, None)
        if el is None:
            return False
        ok = el.set_state(Gst.State.READY) != Gst.StateChangeReturn.FAILURE
        el.set_state(Gst.State.NULL)
        return ok
    except Exception:  # noqa: BLE001
        return False


def set_props(el, props):
    """存在するプロパティだけ設定する（GStreamer の版で名前が違うため）"""
    done = []
    for k, v in props:
        if el.find_property(k) is None:
            continue
        try:
            if isinstance(v, str):
                Gst.util_set_object_arg(el, k, v)
            else:
                el.set_property(k, v)
            done.append("%s=%s" % (k, v))
        except Exception:  # noqa: BLE001
            pass
    return done


def encoder_props(name, kbps, fps):
    gop = max(1, int(round(fps * 2)))
    if name == "nvh264enc":
        return [("preset", "low-latency-hq"), ("rc-mode", "cbr"), ("bitrate", kbps), ("max-bitrate", kbps),
                ("gop-size", gop), ("bframes", 0), ("zerolatency", True), ("rc-lookahead", 0), ("aud", False)]
    if name in ("nvcudah264enc", "nvautogpuh264enc"):
        return [("preset", "p1"), ("tune", "ultra-low-latency"), ("rate-control", "cbr"), ("bitrate", kbps),
                ("gop-size", gop), ("b-frames", 0), ("zero-reorder-delay", True), ("rc-lookahead", 0)]
    if name in ("vah264lpenc", "vah264enc"):
        return [("rate-control", "cbr"), ("bitrate", kbps), ("key-int-max", gop), ("b-frames", 0),
                ("target-usage", 7), ("ref-frames", 1)]
    if name == "vaapih264enc":
        return [("rate-control", "cbr"), ("bitrate", kbps), ("keyframe-period", gop), ("max-bframes", 0)]
    if name == "qsvh264enc":
        return [("rate-control", "cbr"), ("bitrate", kbps), ("gop-size", gop), ("b-frames", 0), ("target-usage", 7)]
    if name == "x264enc":
        return [("tune", "zerolatency"), ("speed-preset", "ultrafast"), ("bitrate", kbps), ("key-int-max", gop),
                ("bframes", 0), ("sliced-threads", True), ("vbv-buf-capacity", 300)]
    if name == "openh264enc":
        return [("bitrate", kbps * 1000), ("gop-size", gop), ("complexity", "low"), ("rate-control", "bitrate")]
    return []


def encoder_profiles(name):
    """圧縮の出力で選べる profile の一覧（無ければ空）"""
    f = Gst.ElementFactory.find(name)
    if f is None:
        return []
    for t in f.get_static_pad_templates():
        if t.direction != Gst.PadDirection.SRC:
            continue
        txt = t.get_caps().to_string()
        m = re.search(r"profile=\(string\)\{([^}]*)\}", txt)
        if m:
            return [p.strip() for p in m.group(1).split(",")]
        m = re.search(r"profile=\(string\)([\w-]+)", txt)
        if m:
            return [m.group(1)]
    return []


# ---------------------------------------------------------------- SDP
PROFILE_PREF = [("42e0", "constrained-baseline"), ("4200", "baseline"), ("640c", "constrained-high"),
                ("4d00", "main"), ("6400", "high")]


def pick_h264_pt(sdp_text):
    """viewer の offer から H.264 の payload type と profile-level-id を選ぶ。
    Chrome 相手では constrained-baseline (42e01f) の行を選ぶと安定する
    （main の行を選ぶとつながらないことがあった）。中身が High でも受け側は再生できる。"""
    pref = PROFILE_PREF
    rtpmap = {}
    fmtp = {}
    for line in sdp_text.splitlines():
        m = re.match(r"a=rtpmap:(\d+) ([\w-]+)/90000", line)
        if m:
            rtpmap[int(m.group(1))] = m.group(2).upper()
        m = re.match(r"a=fmtp:(\d+) (.*)", line)
        if m:
            fmtp[int(m.group(1))] = m.group(2)
    cands = []
    for pt, codec in rtpmap.items():
        if codec != "H264":
            continue
        f = fmtp.get(pt, "")
        mode = re.search(r"packetization-mode=(\d)", f)
        if not mode or mode.group(1) != "1":
            continue
        plid = (re.search(r"profile-level-id=([0-9a-fA-F]{6})", f) or [None, "42e01f"])[1].lower()
        rank = next((i for i, (p, _) in enumerate(pref) if plid.startswith(p)), 9)
        cands.append((rank, pt, plid))
    if not cands:
        return None
    cands.sort()
    _, pt, plid = cands[0]
    prof = next((name for p, name in PROFILE_PREF if plid.startswith(p)), "constrained-baseline")
    return pt, plid, prof


# ---------------------------------------------------------------- session
class Sender:
    def __init__(self, args):
        self.args = args
        self.loop = GLib.MainLoop()
        self.pipe = None
        self.wb = None
        self.session = 0
        self.answer_sent = False
        self.candidates = []
        self.frames = 0
        self.prev = None
        self.started_at = 0
        self.failed_encoders = set()
        if args.passthrough:
            args.input_format = "h264"
            self.encoder = "passthrough"
        else:
            self.encoder = pick_encoder(args.encoder)
        self.src_desc, self.src_info = self.build_source()
        if args.passthrough:
            log("圧縮: しない（カメラが圧縮した H.264 をそのまま送る・試験的）")
        else:
            log("圧縮: %s（%s）" % (self.encoder, "ハード" if self.encoder in HARDWARE else "ソフト"))

    # ---- source
    def build_source(self):
        a = self.args
        if a.test:
            desc = ("videotestsrc is-live=true pattern=smpte ! video/x-raw,width=%d,height=%d,framerate=%s" %
                    (a.width, a.height, fps_fraction(a.fps)))
            if has("timeoverlay"):
                desc += ' ! timeoverlay font-desc="Sans 48" halignment=center valignment=center'
            else:
                desc = desc.replace("pattern=smpte", "pattern=ball")
            if a.passthrough:  # カメラの代わりに、カメラ内の圧縮を真似る（High profile）
                desc += (" ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 bitrate=%d"
                         " ! video/x-h264,profile=high" % int(a.mbps * 1000))
            return desc, "試験映像 %dx%d %sfps" % (a.width, a.height, a.fps)
        cams = list_cameras()
        if not cams:
            raise SystemExit("カメラが見つかりません。X5 を USB でつなぎ、USB モードを「ウェブカメラ」にしてください（v4l2-ctl --list-devices でも確認）")
        if a.device != "auto":
            cam = next((c for c in cams if c[1] == a.device), None) or (a.device, a.device, None)
        else:
            cam = next((c for c in cams if re.search(r"insta360|360|theta", c[0], re.I)), None) or cams[0]
        name, path, caps = cam
        mode = pick_mode(caps, a.width, a.height, a.fps, a.input_format) if caps is not None else None
        if mode is None:
            mode = ("mjpeg" if a.input_format in ("auto", "mjpeg") else a.input_format, a.width, a.height, a.fps)
        fmt, w, h, fps = mode
        if a.passthrough and not fmt.startswith("h264"):
            raise SystemExit("--passthrough はカメラが H.264 を出すときだけ使えます（このカメラの形式は --list で確認）")
        rate = fps_fraction(fps)
        base = fmt.split("/")[0]
        src = "v4l2src device=%s do-timestamp=true" % path
        if base == "raw":
            pix = fmt.split("/")[1] if "/" in fmt else None
            caps_s = "video/x-raw,%swidth=%d,height=%d,framerate=%s" % (("format=%s," % pix) if pix else "", w, h, rate)
            desc = "%s ! %s" % (src, caps_s)
        elif base == "mjpeg":
            dec = "jpegdec"
            desc = "%s ! image/jpeg,width=%d,height=%d,framerate=%s ! queue max-size-buffers=2 leaky=downstream ! %s" % (src, w, h, rate, dec)
        elif base == "h264" and a.passthrough:
            desc = "%s ! video/x-h264,width=%d,height=%d,framerate=%s" % (src, w, h, rate)
        elif base == "h264":
            dec = "nvh264dec" if (self.encoder.startswith("nv") and has("nvh264dec")) else ("vah264dec" if has("vah264dec") else "avdec_h264")
            desc = "%s ! video/x-h264,width=%d,height=%d,framerate=%s ! h264parse ! %s" % (src, w, h, rate, dec)
        else:
            raise SystemExit("この形式 (%s) には対応していません。--input-format mjpeg などを指定してください" % fmt)
        ratio_ok = abs(w / float(h) - 2.0) < 0.02
        info = "%s (%s) %s %dx%d %sfps%s" % (name, path, fmt, w, h, fps, "" if ratio_ok else "  ※2:1 ではない。PICO では歪む")
        self.args.fps, self.args.width, self.args.height = fps, w, h
        return desc, info

    # ---- pipeline
    def build_pipeline(self, pt, profile):
        a = self.args
        kbps = int(a.mbps * 1000)
        if a.passthrough:
            desc = (
                "%s ! h264parse name=enc config-interval=-1 ! queue max-size-buffers=30 leaky=downstream ! "
                "rtph264pay name=pay config-interval=-1 mtu=1200 ! "
                "application/x-rtp,media=video,encoding-name=H264,payload=%d,clock-rate=90000 ! "
                "webrtcbin name=wb bundle-policy=max-bundle"
            ) % (self.src_desc, pt)
            pipe = Gst.parse_launch(desc)
            set_props(pipe.get_by_name("pay"), [("aggregate-mode", "zero-latency")])
            pipe.get_by_name("enc").get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, self._count_frame)
            return pipe
        conv = "videoconvert"
        if a.scale and a.scale != 1.0:
            ow, oh = int(a.width / a.scale) // 2 * 2, int(a.height / a.scale) // 2 * 2
            conv += " ! videoscale ! video/x-raw,width=%d,height=%d" % (ow, oh)
        profs = encoder_profiles(self.encoder)
        use_prof = profile if profile in profs else ("constrained-baseline" if "constrained-baseline" in profs else
                                                     ("baseline" if "baseline" in profs else None))
        enc_caps = "video/x-h264,profile=%s" % use_prof if use_prof else "video/x-h264"
        desc = (
            "%s ! queue max-size-buffers=1 leaky=downstream ! %s ! %s name=enc ! %s ! h264parse ! "
            "rtph264pay name=pay config-interval=-1 mtu=1200 ! "
            "application/x-rtp,media=video,encoding-name=H264,payload=%d,clock-rate=90000 ! "
            "webrtcbin name=wb bundle-policy=max-bundle"
        ) % (self.src_desc, conv, self.encoder, enc_caps, pt)
        pipe = Gst.parse_launch(desc)
        enc = pipe.get_by_name("enc")
        done = set_props(enc, encoder_props(self.encoder, kbps, a.fps))
        set_props(pipe.get_by_name("pay"), [("aggregate-mode", "zero-latency")])
        log("圧縮の設定: %s %s profile=%s" % (self.encoder, " ".join(done), use_prof or "既定"))
        enc.get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, self._count_frame)
        return pipe

    def _count_frame(self, pad, info):
        self.frames += 1
        return Gst.PadProbeReturn.OK

    def stop(self):
        if self.pipe is not None:
            self.pipe.set_state(Gst.State.NULL)
            self.pipe = None
            self.wb = None

    def start_session(self, session, desc):
        self.stop()
        self.session = session
        self.answer_sent = False
        self.candidates = []
        self.prev = None
        sdp_text = desc["sdp"]
        chosen = pick_h264_pt(sdp_text)
        if chosen is None:
            log("PICO の offer に H.264 がありません (session %d)" % session)
            return False
        pt, plid, prof = chosen
        log("PICO から接続要求 (session %d)。H.264 pt=%d profile-level-id=%s" % (session, pt, plid))
        try:
            self.pipe = self.build_pipeline(pt, prof)
        except GLib.Error as e:
            log("パイプラインを作れません: %s" % e.message)
            return False
        self.wb = self.pipe.get_by_name("wb")
        bus = self.pipe.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus, session)
        self.wb.connect("on-ice-candidate", self._on_ice_candidate)
        self.wb.connect("notify::ice-gathering-state", self._on_gathering)
        self.wb.connect("notify::connection-state", self._on_conn_state)
        tr = self.wb.emit("get-transceiver", 0)
        if tr is not None:
            tr.set_property("direction", GstWebRTC.WebRTCRTPTransceiverDirection.SENDONLY)
            # viewer の offer にある H.264 の行に合わせる（profile-level-id は相手の値をそのまま使う）
            tr.set_property("codec-preferences", Gst.Caps.from_string(
                "application/x-rtp,media=video,encoding-name=H264,payload=%d,clock-rate=90000,"
                "packetization-mode=(string)1" % pt))
        self.pipe.set_state(Gst.State.PLAYING)
        self.started_at = time.time()

        ok, msg = GstSdp.SDPMessage.new()
        GstSdp.sdp_message_parse_buffer(sdp_text.encode("utf-8"), msg)
        offer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.OFFER, msg)
        self.wb.emit("set-remote-description", offer, Gst.Promise.new_with_change_func(self._on_remote_set, session))
        return False

    def _on_remote_set(self, promise, session, *_):
        if session != self.session or self.wb is None:
            return
        self.wb.emit("create-answer", None, Gst.Promise.new_with_change_func(self._on_answer, session))

    def _on_answer(self, promise, session, *_):
        if session != self.session or self.wb is None:
            return
        reply = promise.get_reply()
        answer = reply.get_value("answer") if reply else None
        if answer is None:
            log("answer を作れませんでした (session %d)" % session)
            return
        self.wb.emit("set-local-description", answer, None)
        # 候補集めが終わらない場合に備えて 2 秒で送る
        GLib.timeout_add(2000, self._send_answer, session)

    def _on_ice_candidate(self, wb, mline, cand):
        self.candidates.append((mline, cand))

    def _on_gathering(self, wb, pspec):
        st = wb.get_property("ice-gathering-state")
        if st == GstWebRTC.WebRTCICEGatheringState.COMPLETE:
            GLib.idle_add(self._send_answer, self.session)

    def _send_answer(self, session):
        if session != self.session or self.wb is None or self.answer_sent:
            return False
        local = self.wb.get_property("local-description")
        if local is None:
            return False
        sdp = local.sdp.as_text()
        if "a=candidate" not in sdp and self.candidates:
            sdp = add_candidates(sdp, self.candidates)
        self.answer_sent = True
        threading.Thread(target=self._post_answer, args=(session, sdp), daemon=True).start()
        return False

    def _post_answer(self, session, sdp):
        try:
            st, _ = http_json("POST", "/api/answer", {"session": session, "desc": {"type": "answer", "sdp": sdp}})
            n = sdp.count("a=candidate")
            if st == 200:
                log("PICO に応答しました (session %d, 候補 %d 個)" % (session, n))
            else:
                log("応答が古くなっていました (session %d)" % session)
        except Exception as e:  # noqa: BLE001
            log("応答を送れません: %s" % e)

    def _on_conn_state(self, wb, pspec):
        st = wb.get_property("connection-state")
        name = st.value_nick if hasattr(st, "value_nick") else str(st)
        log("接続状態 %s (session %d)" % (name, self.session))
        if st in (GstWebRTC.WebRTCPeerConnectionState.FAILED, GstWebRTC.WebRTCPeerConnectionState.CLOSED):
            GLib.timeout_add(3000, self._stop_if, self.session)  # カメラを放す（次の接続要求で開き直す）

    def _stop_if(self, session):
        if session == self.session:
            self.stop()
        return False

    def _on_bus(self, bus, msg, session):
        t = msg.type
        if t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            log("エラー (session %d): %s / %s" % (session, err.message, (dbg or "").split("\n")[0][:300]))
            if session == self.session:
                self.stop()
                self._fallback_encoder()
        elif t == Gst.MessageType.WARNING:
            w, dbg = msg.parse_warning()
            log("注意: %s" % w.message)

    def _fallback_encoder(self):
        """ハード圧縮が動かなかったら次の候補に切り替える（PICO がつなぎ直したときから使う）"""
        if self.args.passthrough or self.args.encoder != "auto" or self.encoder not in HARDWARE:
            return
        self.failed_encoders.add(self.encoder)
        nxt = next((n for n in ENCODERS if n not in self.failed_encoders and has(n) and _encoder_opens(n)), None)
        if nxt:
            log("圧縮 %s が動かなかったので %s に切り替えます。PICO は数秒でつなぎ直します" % (self.encoder, nxt))
            self.encoder = nxt

    # ---- stats
    def stats_tick(self):
        if self.wb is None:
            return True
        frames = self.frames
        now = time.time()
        promise = Gst.Promise.new_with_change_func(self._on_stats, (frames, now))
        self.wb.emit("get-stats", None, promise)
        return True

    def _on_stats(self, promise, mark, *_):
        frames, now = mark
        reply = promise.get_reply()
        if reply is None:
            return
        sent = [0]
        rtt = [None]

        def visit(field_id, value, *__):
            if isinstance(value, Gst.Structure):
                typ = value.get_value("type") if value.has_field("type") else None
                nick = typ.value_nick if hasattr(typ, "value_nick") else str(typ)
                if "outbound" in nick and value.has_field("bytes-sent"):
                    sent[0] += value.get_value("bytes-sent")
                if "remote-inbound" in nick and value.has_field("round-trip-time"):
                    rtt[0] = value.get_value("round-trip-time")
            return True

        try:
            reply.foreach(visit, None)
        except TypeError:
            reply.foreach(visit)
        cur = (now, frames, sent[0])
        if self.prev:
            dt = max(1e-3, cur[0] - self.prev[0])
            fps = (cur[1] - self.prev[1]) / dt
            mbps = (cur[2] - self.prev[2]) * 8 / dt / 1e6
            line = "送信: %s / %.0ffps / %.1fMbps / %s" % (self.out_size(), fps, mbps, self.encoder)
            if rtt[0] is not None:
                line += " / RTT%dms" % round(rtt[0] * 1000)
            log(line)
        self.prev = cur

    def out_size(self):
        a = self.args
        if a.scale and a.scale != 1.0:
            return "%dx%d" % (int(a.width / a.scale) // 2 * 2, int(a.height / a.scale) // 2 * 2)
        return "%dx%d" % (a.width, a.height)

    # ---- signaling
    def offer_loop(self):
        last = 0
        warned = False
        while True:
            try:
                st, body = http_json("GET", "/api/offer?after=%d" % last, timeout=15)
                warned = False
                if st == 200 and body:
                    last = body["session"]
                    GLib.idle_add(self.start_session, body["session"], body["desc"])
            except Exception as e:  # noqa: BLE001
                if not warned:
                    log("サーバー (%s) とつながりません。pano_server.py は動いていますか: %s" % (_server, e), to_server=False)
                    warned = True
                time.sleep(2)

    def run(self):
        log("カメラ: %s" % self.src_info)
        log("PICO の接続を待っています（PICO のブラウザで https://<このPCのIP>:8443/ を開く）")
        threading.Thread(target=self.offer_loop, daemon=True).start()
        GLib.timeout_add_seconds(5, self.stats_tick)
        try:
            self.loop.run()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
            log("終了します", to_server=False)


def add_candidates(sdp, candidates):
    """local-description に候補が入っていない GStreamer 向け: m= ごとに a=candidate を足す"""
    lines = sdp.replace("\r\n", "\n").split("\n")
    out = []
    mline = -1
    by_m = {}
    for m, c in candidates:
        by_m.setdefault(m, []).append(c)

    def flush(idx):
        for c in by_m.get(idx, []):
            out.append("a=" + c if not c.startswith("a=") else c)
        if idx in by_m:
            out.append("a=end-of-candidates")

    for line in lines:
        if line.startswith("m="):
            if mline >= 0:
                flush(mline)
            mline += 1
        if line:
            out.append(line)
    if mline >= 0:
        flush(mline)
    return "\r\n".join(out) + "\r\n"


def print_list():
    print("== カメラ ==")
    cams = list_cameras()
    if not cams:
        print("  見つかりません（X5 の USB モードを「ウェブカメラ」に。v4l2-ctl --list-devices も見てください）")
    for name, path, caps in cams:
        print("  %s  %s" % (path, name))
        for fmt, w, h, rates in caps_modes(caps):
            mark = "  ← 2:1" if abs(w / float(h) - 2.0) < 0.02 else ""
            print("      %-10s %5dx%-5d %s fps%s" % (fmt, w, h, ",".join("%g" % r for r in rates) or "?", mark))
    print("== H.264 の圧縮 ==")
    for n in ENCODERS:
        if has(n):
            ok = _encoder_opens(n)
            print("  %-18s %s %s" % (n, "使える" if ok else "あるが開けない（GPU/ドライバー）", "（ハード）" if n in HARDWARE else "（ソフト）"))
    print("  自動で選ぶもの: %s" % next((n for n in ENCODERS if has(n) and _encoder_opens(n)), "なし"))
    print("== WebRTC ==")
    for n in ("webrtcbin", "nicesrc", "rtph264pay", "jpegdec", "h264parse"):
        print("  %-12s %s" % (n, "あり" if has(n) else "なし ← 要インストール"))


def main():
    global _log_file, _server
    ap = argparse.ArgumentParser(description="360度カメラ → PICO を GStreamer（ハード圧縮）で送る")
    ap.add_argument("--server", default="http://127.0.0.1:8080", help="pano_server.py の http（PC 内）")
    ap.add_argument("--device", default="auto", help="/dev/videoN（auto なら Insta360 / THETA を探す）")
    ap.add_argument("--width", type=int, default=2880)
    ap.add_argument("--height", type=int, default=1440)
    ap.add_argument("--fps", type=float, default=30)
    ap.add_argument("--input-format", default="auto", choices=["auto", "raw", "mjpeg", "h264"],
                    help="カメラから受け取る形式（auto: 無圧縮 > MJPEG > H.264 の順）")
    ap.add_argument("--scale", type=float, default=1.0, help="送る大きさを 1/scale に（1.5 で 1920x960）")
    ap.add_argument("--mbps", type=float, default=15, help="ビットレート (Mbps)")
    ap.add_argument("--encoder", default="auto", help="auto / nvh264enc / vah264enc / x264enc など")
    ap.add_argument("--passthrough", action="store_true",
                    help="PC で圧縮し直さず、カメラの H.264 をそのまま送る（カメラが H.264 を出すときだけ。試験的）")
    ap.add_argument("--test", action="store_true", help="カメラの代わりに試験映像を送る")
    ap.add_argument("--list", action="store_true", help="カメラと圧縮の一覧を出して終わる")
    ap.add_argument("--log-dir", default=os.path.join(HERE, "logs"))
    args = ap.parse_args()

    Gst.init(None)
    if args.list:
        print_list()
        return
    os.makedirs(args.log_dir, exist_ok=True)
    path = os.path.join(args.log_dir, "gst_sender_%s.log" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    _log_file = open(path, "a", encoding="utf-8")
    _server = args.server.rstrip("/")
    log("ログ: %s / GStreamer %s" % (path, Gst.version_string()), to_server=False)
    Sender(args).run()


if __name__ == "__main__":
    main()
