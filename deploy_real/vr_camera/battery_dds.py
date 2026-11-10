# -*- coding: utf-8 -*-
"""
G1 のバッテリー残量を読むだけ (DDS の rt/lf/bmsstate、unitree_hg BmsState_ の soc)。何も送らない。

映像の右上に小さく「電池 51%」と出すために、カメラの送信プログラムの中で使う。テレオペ (腕の IK・ブリッジ)
が動いていなくても読める。2026-11-10 に G1 の /usr/bin/python3 (3.8) で、環境変数なしで読めることを確かめた
(read_lowstate.py: soc 51、bmsvoltage [48737, 48774, 0] mV、current -1965)。

unitree_sdk2py が無い・DDS を始められないときは、その旨を一度だけ書いて何もしない (映像は止めない)。
"""

import threading
import time


class BatteryReader(object):
    """BmsState_ の最新の soc [%] と受け取った時刻を持つ"""

    def __init__(self, net="eth0", topic="rt/lf/bmsstate", log=print, fake=None):
        self.net = net
        self.topic = topic
        self.log = log
        self.fake = fake          # テスト用: DDS を使わず、この値を出す
        self.lock = threading.Lock()
        self.soc = None
        self.t = 0.0
        self.volt = None
        self.ok = False           # 読み始められたか (False なら電池は出さない)
        self.sub = None
        self.logged_first = False

    def start(self):
        if self.fake is not None:
            self.ok = True
            self.log("電池表示: テスト用の値 %d%% を出します (DDS は使いません)" % self.fake)
            return True
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg import dds_ as hg
        except Exception as e:
            self.log("電池表示: unitree_sdk2py を読み込めません (%s)。電池は出しません" % e)
            return False
        if not hasattr(hg, "BmsState_"):
            self.log("電池表示: unitree_sdk2py に BmsState_ がありません。電池は出しません")
            return False
        try:
            ChannelFactoryInitialize(0, self.net)
            self.sub = ChannelSubscriber(self.topic, hg.BmsState_)
            self.sub.Init(self._on_msg, 10)
        except Exception as e:
            self.log("電池表示: DDS (%s, %s) を始められません (%s)。電池は出しません" % (self.net, self.topic, e))
            return False
        self.ok = True
        self.log("電池表示: %s を %s で読みます (読むだけ)" % (self.topic, self.net))
        return True

    def _on_msg(self, m):
        try:
            soc = int(m.soc)
            mv = [int(v) for v in m.bmsvoltage if int(v) > 0]
        except Exception:
            return
        with self.lock:
            self.soc, self.t = soc, time.time()
            self.volt = (mv[0] / 1000.0) if mv else None
        if not self.logged_first:
            self.logged_first = True
            self.log("電池表示: 最初の値 %d%%%s" % (soc, " (%.1f V)" % self.volt if self.volt else ""))

    def latest(self):
        """(soc または None, 受け取った時刻)"""
        if self.fake is not None:
            return int(self.fake), time.time()
        with self.lock:
            return self.soc, self.t
