#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
シミュレーター (MuJoCo) の中の G1 の「頭のカメラ (D435i) の目線」を描画して、g1_cam_to_pico.py に渡す。

  シミュレーター (arm_sim.py など) が Redis に書いている関節角 (既定キー sim_qpos) を読むだけ。書き込みはしない。
  → 同じ G1 モデルに頭カメラを足したものに反映 → GPU (EGL) で描画
  → 127.0.0.1:5601 で待つ g1_cam_to_pico.py (--source sim) に RGBA の生フレームを渡す
  → g1_cam_to_pico.py がハードウェア H.264 にして PICO へ

カメラの位置・向きは Unitree の URDF (g1_29dof_rev_1_0.urdf の d435_joint) と同じ:
  torso_link から xyz = (0.0576235, 0.01753, 0.42987), 下向き pitch = 0.8307767 rad, 縦の画角 約 42 度。

gmr 環境の python (mujoco, redis, numpy あり) で動かす:
  MUJOCO_GL=egl ~/miniforge3/envs/gmr/bin/python sim_head_cam.py --model <xml>
確認用:
  ... sim_head_cam.py --model <xml> --png /tmp/head.png       # 1 枚だけ描いて保存
  ... sim_head_cam.py --model <xml> --bench 100               # 描画時間を測る
"""

import argparse
import json
import math
import os
import socket
import struct
import sys
import time
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np  # noqa: E402
import mujoco  # noqa: E402

MAGIC = b"G1RAW1"
FMT_RGBA = 1

D435_POS = (0.0576235, 0.01753, 0.42987)
D435_PITCH = 0.8307767239493009
HIDE_GROUP = 5


def log(msg):
    t = time.strftime("%H:%M:%S") + ".%03d" % int((time.time() % 1) * 1000)
    print("[%s] %s" % (t, msg), flush=True)


# ---------------------------------------------------------------------------
# モデル: 元の XML に頭カメラを足す (元のファイルは変更しない)
# ---------------------------------------------------------------------------
def _absolutize(root, base_dir):
    comp = root.find("compiler")
    if comp is None:
        comp = ET.SubElement(root, "compiler")
    for key in ("meshdir", "texturedir", "assetdir"):
        v = comp.get(key)
        if v is not None and not os.path.isabs(v):
            comp.set(key, os.path.normpath(os.path.join(base_dir, v)))
    if comp.get("meshdir") is None and comp.get("assetdir") is None:
        comp.set("meshdir", base_dir)
    for inc in root.iter("include"):
        f = inc.get("file")
        if f and not os.path.isabs(f):
            inc.set("file", os.path.normpath(os.path.join(base_dir, f)))


def build_model(xml_path, body, pos, pitch, fovy, add_floor, hide_meshes=("head_link", "logo_link")):
    xml_path = os.path.abspath(os.path.expanduser(xml_path))
    tree = ET.parse(xml_path)
    root = tree.getroot()
    _absolutize(root, os.path.dirname(xml_path))

    target = None
    for b in root.iter("body"):
        if b.get("name") == body:
            target = b
            break
    if target is None:
        raise RuntimeError("モデルに body '%s' がありません" % body)
    # MuJoCo のカメラは -z 方向を見て、x=画面の右、y=画面の上。
    # 体の座標 (x 前, y 左, z 上) で、前を向いて pitch だけ下を向く。
    s, c = math.sin(pitch), math.cos(pitch)
    cam = ET.Element("camera", {
        "name": "head_d435",
        "pos": "%.6f %.6f %.6f" % pos,
        "xyaxes": "0 -1 0 %.6f 0 %.6f" % (s, c),
        "fovy": "%.2f" % fovy,
    })
    target.insert(0, cam)

    # カメラは頭の中にあるので、頭の殻 (head_link / logo_link) が視界をふさぐ。
    # それらの geom を group 5 に移し、描画のときに group 5 を消す。
    hidden = 0
    for g in root.iter("geom"):
        if g.get("mesh") in hide_meshes:
            g.set("group", str(HIDE_GROUP))
            hidden += 1
    log("頭カメラ: %s に pos=%s pitch=%.3f rad, 隠した geom %d 個" % (body, cam.get("pos"), pitch, hidden))

    if add_floor:
        wb = root.find("worldbody")
        # worldbody が複数に分かれているモデルもあるので、全体から床を探す (二重の床はちらつく)
        has_plane = any(g.get("type") == "plane" for g in root.iter("geom"))
        if not has_plane:
            ET.SubElement(wb, "light", {"pos": "1 0 3.5", "dir": "0 0 -1", "directional": "true"})
            ET.SubElement(wb, "geom", {"name": "headcam_floor", "type": "plane", "size": "0 0 0.05",
                                       "rgba": "0.35 0.4 0.45 1"})
    # 描画用バッファの大きさ (既定は 640x480 なので広げる)
    vis = root.find("visual")
    if vis is None:
        vis = ET.SubElement(root, "visual")
    glob = vis.find("global")
    if glob is None:
        glob = ET.SubElement(vis, "global")
    glob.set("offwidth", "1920")
    glob.set("offheight", "1080")
    q = vis.find("quality")
    if q is None:
        q = ET.SubElement(vis, "quality")
    q.set("shadowsize", "1024")

    xml = ET.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml)


# ---------------------------------------------------------------------------
# Redis の関節角を model の qpos に入れる
# ---------------------------------------------------------------------------
class QposReader(object):
    def __init__(self, model, host, port, key):
        import redis
        self.r = redis.Redis(host=host, port=port)
        self.key = key
        self.model = model
        self.hinge_adr = [model.jnt_qposadr[j] for j in range(model.njnt)
                          if model.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)]
        self.name_adr = {}
        for j in range(model.njnt):
            n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            if n:
                self.name_adr[n] = model.jnt_qposadr[j]
        self.warned = set()
        self.last_ok = 0.0

    def _warn(self, key, msg):
        if key not in self.warned:
            self.warned.add(key)
            log(msg)

    def read(self, qpos):
        """qpos を更新したら True"""
        try:
            raw = self.r.get(self.key)
        except Exception as e:
            self._warn("conn", "Redis を読めません: %s" % e)
            return False
        if raw is None:
            self._warn("none", "Redis に %s がありません (シミュレーターが動いていない?)" % self.key)
            return False
        vals = None
        try:
            obj = json.loads(raw)
        except Exception:
            obj = None
        if isinstance(obj, dict):
            for k in ("joints", "qpos", "q", "data"):
                if k in obj:
                    obj = obj[k]
                    break
        if isinstance(obj, dict):  # 関節名 → 角度
            n = 0
            for name, v in obj.items():
                adr = self.name_adr.get(name)
                if adr is not None:
                    qpos[adr] = float(v)
                    n += 1
            if n == 0:
                self._warn("names", "%s の関節名がモデルと 1 つも合いません" % self.key)
                return False
            self.last_ok = time.time()
            return True
        if isinstance(obj, list):
            vals = np.asarray(obj, dtype=np.float64)
        elif obj is None:
            for dt in (np.float64, np.float32):
                if len(raw) % np.dtype(dt).itemsize == 0:
                    a = np.frombuffer(raw, dtype=dt)
                    if a.size in (self.model.nq, len(self.hinge_adr)):
                        vals = a.astype(np.float64)
                        break
        if vals is None:
            self._warn("fmt", "%s の形式が分かりません (先頭 %r)" % (self.key, raw[:40]))
            return False
        if vals.size == self.model.nq:
            qpos[:] = vals
        elif vals.size == len(self.hinge_adr):
            qpos[self.hinge_adr] = vals
        else:
            self._warn("len", "%s の長さ %d がモデル (nq=%d, 関節 %d) と合いません" % (
                self.key, vals.size, self.model.nq, len(self.hinge_adr)))
            return False
        self.last_ok = time.time()
        return True


# ---------------------------------------------------------------------------
class HeadCam(object):
    def __init__(self, args):
        self.args = args
        self.model = build_model(args.model, args.body, D435_POS if not args.cam_pos else tuple(args.cam_pos),
                                 args.cam_pitch, args.fovy, not args.no_floor)
        self.data = mujoco.MjData(self.model)
        # 腕シム (arm_sim.py) は keyframe・freejoint・脚の関節を消しているので、骨盤は XML の pos (0 0 0.793)、
        # 脚は角度 0 のまま。ここでも keyframe ("home" は骨盤 1.0 m・膝曲げ) は使わず qpos0 から始める。
        mujoco.mj_resetData(self.model, self.data)
        if args.keyframe >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, args.keyframe)
        mujoco.mj_forward(self.model, self.data)
        self.renderer = mujoco.Renderer(self.model, args.height, args.width)
        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[HIDE_GROUP] = 0
        # 影と反射は切る (GPU の負荷を下げる。ソフト描画では床が崩れる原因にもなる)
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = bool(args.shadows)
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        self.rgba = np.empty((args.height, args.width, 4), dtype=np.uint8)
        self.rgba[..., 3] = 255
        self.reader = None
        if not args.no_redis:
            self.reader = QposReader(self.model, args.redis_host, args.redis_port, args.key)
        log("モデル nq=%d, 関節 %d, 描画 %dx%d, MUJOCO_GL=%s" % (
            self.model.nq, self.model.njnt, args.width, args.height, os.environ.get("MUJOCO_GL")))

    def frame(self):
        if self.reader:
            self.reader.read(self.data.qpos)
        mujoco.mj_kinematics(self.model, self.data)
        self.renderer.update_scene(self.data, camera="head_d435", scene_option=self.opt)
        rgb = self.renderer.render()
        self.rgba[..., :3] = rgb
        return self.rgba


def serve(args, cam):
    host, port = args.listen.rsplit(":", 1)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, int(port)))
    srv.listen(1)
    log("待ち受け %s:%s (g1_cam_to_pico.py --source sim がつなぐと描画を始めます)" % (host, port))
    period = 1.0 / args.fps
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        log("接続 %s:%d → 描画開始" % addr)
        n = 0
        t0 = time.time()
        last = t0
        render_sum = 0.0
        render_max = 0.0
        try:
            conn.sendall(MAGIC + struct.pack("<IIII", args.width, args.height, args.fps, FMT_RGBA))
            nxt = time.time()
            while True:
                tr = time.time()
                img = cam.frame()
                dt = time.time() - tr
                render_sum += dt
                render_max = max(render_max, dt)
                conn.sendall(img.tobytes())
                n += 1
                now = time.time()
                if now - last >= args.stats_sec:
                    age = (now - cam.reader.last_ok) if cam.reader and cam.reader.last_ok else -1
                    log("描画 %.1f fps, 1 枚 平均 %.1f ms / 最大 %.1f ms, 関節角の古さ %.2f 秒" % (
                        n / (now - t0), 1000 * render_sum / max(n, 1), 1000 * render_max, age))
                    last = now
                nxt += period
                sl = nxt - time.time()
                if sl > 0:
                    time.sleep(sl)
                else:
                    nxt = time.time()
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            log("切断 (%s)。%d 枚送りました。次の接続を待ちます" % (e, n))
        finally:
            conn.close()


def main():
    p = argparse.ArgumentParser(description="シミュレーターの G1 頭カメラ目線を描画して g1_cam_to_pico.py に渡す")
    p.add_argument("--model", required=True, help="シミュレーターと同じ G1 の MuJoCo XML")
    p.add_argument("--body", default="torso_link", help="カメラを付ける body")
    p.add_argument("--cam-pos", type=float, nargs=3, default=None, help="body からの位置 (既定 D435i)")
    p.add_argument("--cam-pitch", type=float, default=D435_PITCH, help="下向きの角度 rad (既定 D435i)")
    p.add_argument("--fovy", type=float, default=42.0, help="縦の画角 (度)")
    p.add_argument("--keyframe", type=int, default=-1, help="初期姿勢に使う keyframe 番号 (既定 -1 = 使わない。腕シムと同じ)")
    p.add_argument("--no-floor", action="store_true")
    p.add_argument("--shadows", action="store_true", help="影を描く (既定は切る)")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--listen", default="127.0.0.1:5601")
    p.add_argument("--redis-host", default="127.0.0.1")
    p.add_argument("--redis-port", type=int, default=6379)
    p.add_argument("--key", default="sim_qpos", help="シミュレーターの関節角の Redis キー")
    p.add_argument("--no-redis", action="store_true", help="Redis を読まず初期姿勢のまま (確認用)")
    p.add_argument("--stats-sec", type=float, default=5.0)
    p.add_argument("--png", default="", help="1 枚描いて保存して終わる")
    p.add_argument("--bench", type=int, default=0, help="N 枚描いて時間を測って終わる")
    args = p.parse_args()

    cam = HeadCam(args)
    try:
        return run(args, cam)
    finally:
        try:
            cam.renderer.close()
        except Exception:
            pass


def run(args, cam):
    if args.png:
        img = cam.frame()
        try:
            from PIL import Image
            Image.fromarray(img[..., :3]).save(args.png)
        except ImportError:
            import zlib
            h, w = img.shape[:2]
            raw = b"".join(b"\x00" + img[y, :, :3].tobytes() for y in range(h))

            def chunk(t, d):
                return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
            with open(args.png, "wb") as f:
                f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                        + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
        log("保存しました %s" % args.png)
        return 0
    if args.bench:
        cam.frame()
        t0 = time.time()
        ts = []
        for _ in range(args.bench):
            t = time.time()
            cam.frame()
            ts.append(time.time() - t)
        dt = time.time() - t0
        ts = np.array(ts) * 1000
        log("描画 %d 枚: 平均 %.1f ms, 95%% %.1f ms, 最大 %.1f ms (%.1f fps 相当)" % (
            args.bench, ts.mean(), np.percentile(ts, 95), ts.max(), args.bench / dt))
        return 0
    try:
        serve(args, cam)
    except KeyboardInterrupt:
        log("終了")
    return 0


if __name__ == "__main__":
    sys.exit(main())
