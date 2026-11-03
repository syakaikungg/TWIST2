# -*- coding: utf-8 -*-
"""
腕テレオペの状態 (Redis `teleop_display`) を PICO に送る映像に文字で重ねる。

腕の IK (deploy_real/arm_ik/arm_ik_dryrun.py) が 50 Hz で書く JSON を読むだけ (GET のみ、書き込みはしない):
  {"t": 1762176000.0, "level": "ready", "ja": "握れば動きます", "en": "READY - hold grip", ...}
  level: stop / warn / wait / idle / ready / ok

G1 のホストの python3 (3.8) には redis モジュールが無いので、GET だけの小さなクライアントを持つ。
文字は GStreamer の textoverlay (pango) で描く。左右並びのときは、並べる前の片目の絵に描くので両目の同じ位置に出る。
"""

import json
import socket
import subprocess
import threading
import time

try:
    from gi.repository import GLib
except ImportError:  # pragma: no cover
    GLib = None


# level ごとの文字色 (ARGB)。背景は半透明の黒
LEVEL_COLORS = {
    "stop": 0xFFFF4040,   # 赤: 非常停止
    "warn": 0xFFFFA020,   # 橙: 通信待ち
    "wait": 0xFFFFE040,   # 黄: 準備中
    "idle": 0xFFD0D0D0,   # 灰: 一時停止中
    "ready": 0xFF50D8FF,  # 水色: 握れば動く
    "ok": 0xFF60FF70,     # 緑: 追従中
}

# IK からの状態が読めないときの表示 (level, ja, en)
NO_IK = ("stop", "IK停止（状態が届いていません）", "IK STOPPED - no status")
NO_KEY = ("idle", "IK未起動", "IK NOT RUNNING")
NO_REDIS = ("warn", "状態を読めません（Redis）", "NO STATUS (redis)")
BAD_DATA = ("warn", "状態の形式が不明", "STATUS UNREADABLE")


def has_japanese_font():
    """fontconfig に日本語を描けるフォントがあるか"""
    try:
        out = subprocess.check_output(["fc-list", ":lang=ja", "family"], stderr=subprocess.DEVNULL,
                                      timeout=5)
        fams = [l.strip() for l in out.decode("utf-8", "replace").splitlines() if l.strip()]
        return fams[0] if fams else ""
    except Exception:
        return ""


class MiniRedis(object):
    """Redis の GET だけを話す最小のクライアント (RESP)"""

    def __init__(self, host="127.0.0.1", port=6379, timeout=0.3):
        self.addr = (host, int(port))
        self.timeout = timeout
        self.sock = None
        self.f = None

    def _connect(self):
        s = socket.create_connection(self.addr, timeout=self.timeout)
        s.settimeout(self.timeout)
        self.sock = s
        self.f = s.makefile("rb")

    def close(self):
        for o in (self.f, self.sock):
            try:
                if o:
                    o.close()
            except Exception:
                pass
        self.sock = self.f = None

    def get(self, key):
        """値 (bytes) を返す。キーが無ければ None。つながらなければ例外"""
        if self.sock is None:
            self._connect()
        k = key.encode("utf-8")
        try:
            self.sock.sendall(b"*2\r\n$3\r\nGET\r\n$%d\r\n%s\r\n" % (len(k), k))
            line = self.f.readline()
            if not line:
                raise ConnectionError("Redis が接続を閉じました")
            if line[:1] == b"$":
                n = int(line[1:].strip())
                if n < 0:
                    return None
                data = self.f.read(n + 2)
                if len(data) < n + 2:
                    raise ConnectionError("Redis からの返事が途中で切れました")
                return data[:n]
            if line[:1] == b"-":
                raise RuntimeError("Redis: %s" % line[1:].strip().decode("utf-8", "replace"))
            raise RuntimeError("Redis の返事が想定と違います: %r" % line[:40])
        except Exception:
            self.close()
            raise


def describe(raw, now, lang, stale_sec):
    """teleop_display の値から (level, 表示する文字) を決める"""
    def pick(t):
        return t[1] if lang == "ja" else t[2]
    if raw is None:
        return NO_KEY[0], pick(NO_KEY)
    try:
        d = json.loads(raw.decode("utf-8"))
        t = float(d.get("t", 0))
        level = str(d.get("level", ""))
        text = d.get("ja") if lang == "ja" else d.get("en")
        if not text:
            text = d.get("en") or d.get("ja") or level
    except Exception:
        return BAD_DATA[0], pick(BAD_DATA)
    age = now - t
    if age > stale_sec:
        lv, ja, en = NO_IK
        if lang == "ja":
            return lv, "%s %.0f秒" % (ja, min(age, 999))
        return lv, "%s %.0fs" % (en, min(age, 999))
    return level, str(text)


class StatusOverlay(object):
    """teleop_display を読み続けて textoverlay の文字と色を変える (配信 1 回ごとに作る)"""

    def __init__(self, element, args, log=print):
        self.el = element
        self.log = log
        self.key = args.overlay_key
        self.lang = args.overlay_lang
        self.stale_sec = args.overlay_stale
        self.period = 1.0 / max(1.0, args.overlay_hz)
        self.redis = MiniRedis(args.redis_host, args.redis_port)
        self.stop_event = threading.Event()
        self.thread = None
        self.level = ""
        self.text = ""
        self.redis_ok = None
        self.changes = 0

    def start(self):
        # 古い GStreamer に無いかもしれない見た目の設定は、無ければ飛ばす
        for name, val in (("shading-value", 150), ("draw-shadow", False)):
            try:
                self.el.set_property(name, val)
            except Exception:
                pass
        self._apply(*self._read())
        self.thread = threading.Thread(target=self._run, name="overlay", daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
        self.redis.close()

    def _read(self):
        try:
            raw = self.redis.get(self.key)
            if self.redis_ok is not True:
                self.log("状態表示: Redis %s:%d の %s を読みます" % (
                    self.redis.addr[0], self.redis.addr[1], self.key))
            self.redis_ok = True
        except Exception as e:
            if self.redis_ok is not False:
                self.log("状態表示: Redis を読めません (%s)。つながるまで %s と表示します" % (
                    e, NO_REDIS[1] if self.lang == "ja" else NO_REDIS[2]))
            self.redis_ok = False
            return NO_REDIS[0], (NO_REDIS[1] if self.lang == "ja" else NO_REDIS[2])
        return describe(raw, time.time(), self.lang, self.stale_sec)

    def _apply(self, level, text):
        if level == self.level and text == self.text:
            return
        self.level, self.text = level, text
        self.changes += 1
        shown = GLib.markup_escape_text(text) if GLib else text
        try:
            self.el.set_property("color", LEVEL_COLORS.get(level, 0xFFFFFFFF))
            self.el.set_property("text", shown)
        except Exception as e:
            self.log("状態表示を更新できません: %s" % e)

    def _run(self):
        while not self.stop_event.wait(self.period):
            self._apply(*self._read())

