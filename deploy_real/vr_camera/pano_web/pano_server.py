#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
360度カメラ (Insta360 のウェブカメラモードなど) の映像を、PICO のブラウザ (WebXR) で
見回せるように流すための小さなサーバー。

構成:
  PC の Chrome (sender.html)  --WebRTC 映像-->  PICO のブラウザ (viewer.html, WebXR で球に貼る)
  このサーバーは「ページの配信」と「つなぐための SDP の受け渡し」だけをする（映像は通らない）。

  - http://localhost:<http-port>/sender.html   … PC で開く送信ページ（localhost なので証明書なしでカメラが使える）
  - https://<PCのIP>:<https-port>/              … PICO で開く表示ページ（WebXR は https が必要。自己署名の証明書）

標準ライブラリだけで動く（Python 3.8 以上）。証明書は初回に自動で作る
（cryptography モジュールか openssl コマンドのどちらかが必要）。
"""

import argparse
import datetime
import ipaddress
import json
import os
import shutil
import socket
import ssl
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC_FILES = {
    "/sender.html": "text/html; charset=utf-8",
    "/clock.html": "text/html; charset=utf-8",
    "/viewer.html": "text/html; charset=utf-8",
    "/sender.js": "text/javascript; charset=utf-8",
    "/viewer.js": "text/javascript; charset=utf-8",
    "/common.js": "text/javascript; charset=utf-8",
    "/three.module.min.js": "text/javascript; charset=utf-8",
}
LONG_POLL_SEC = 10.0
VIEWER_URLS = []  # 起動時に埋める（送信ページに表示する）

_log_lock = threading.Lock()
_log_file = None


def log(msg):
    line = "[%s] %s" % (datetime.datetime.now().strftime("%H:%M:%S"), msg)
    with _log_lock:
        print(line, flush=True)
        if _log_file is not None:
            _log_file.write(line + "\n")
            _log_file.flush()


# ---------------------------------------------------------------- signaling state
class Signaling:
    """viewer(PICO) が offer を出し、sender(PC) が answer を返す。1 対 1、最後の viewer が勝つ。"""

    def __init__(self):
        self.cond = threading.Condition()
        self.session = 0
        self.offer = None
        self.answer = None
        self.last_seen = {"sender": 0.0, "viewer": 0.0}

    def touch(self, role):
        if role in self.last_seen:
            self.last_seen[role] = time.time()

    def put_offer(self, desc):
        with self.cond:
            self.session += 1
            self.offer = desc
            self.answer = None
            self.cond.notify_all()
            return self.session

    def wait_offer(self, after, timeout):
        """after より新しく、まだ answer の付いていない offer を待つ"""
        with self.cond:
            self.cond.wait_for(
                lambda: self.session > after and self.offer is not None and self.answer is None,
                timeout)
            if self.session > after and self.offer is not None and self.answer is None:
                return self.session, self.offer
            return None, None

    def put_answer(self, session, desc):
        with self.cond:
            if session != self.session:
                return False
            self.answer = desc
            self.cond.notify_all()
            return True

    def wait_answer(self, session, timeout):
        with self.cond:
            self.cond.wait_for(lambda: self.session != session or self.answer is not None, timeout)
            if self.session != session:
                return "stale", None
            if self.answer is not None:
                return "ok", self.answer
            return "wait", None

    def status(self):
        now = time.time()
        with self.cond:
            return {
                "session": self.session,
                "has_offer": self.offer is not None,
                "has_answer": self.answer is not None,
                "sender_seen_sec": round(now - self.last_seen["sender"], 1) if self.last_seen["sender"] else None,
                "viewer_seen_sec": round(now - self.last_seen["viewer"], 1) if self.last_seen["viewer"] else None,
            }


SIG = Signaling()


# ---------------------------------------------------------------- HTTP handler
class Handler(BaseHTTPRequestHandler):
    server_version = "pano_web/1.0"
    index_page = "/viewer.html"  # サーバーごとに上書き

    def log_message(self, fmt, *args):  # 既定のアクセスログは出さない（多すぎる）
        pass

    def _send(self, code, body=b"", ctype="application/json; charset=utf-8", extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        if body and self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, code, obj):
        self._send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"))

    def _read_json(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0 or n > 1_000_000:
            raise ValueError("bad body length %d" % n)
        return json.loads(self.rfile.read(n).decode("utf-8"))

    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        note_client(self.client_address[0], "ページの読み込みあり")
        u = urlparse(self.path)
        q = parse_qs(u.query)
        path = u.path
        if path == "/":
            self._send(302, extra={"Location": self.index_page})
            return
        if path in STATIC_FILES:
            fp = os.path.join(HERE, path.lstrip("/"))
            try:
                with open(fp, "rb") as f:
                    body = f.read()
            except OSError:
                self._send(404, b"not found", "text/plain")
                return
            self._send(200, body, STATIC_FILES[path])
            return
        if path == "/api/offer":  # sender が待つ
            SIG.touch("sender")
            after = int((q.get("after") or ["0"])[0])
            sid, offer = SIG.wait_offer(after, LONG_POLL_SEC)
            SIG.touch("sender")
            if sid is None:
                self._send(204)
            else:
                self._json(200, {"session": sid, "desc": offer})
            return
        if path == "/api/answer":  # viewer が待つ
            SIG.touch("viewer")
            sid = int((q.get("session") or ["0"])[0])
            st, ans = SIG.wait_answer(sid, LONG_POLL_SEC)
            SIG.touch("viewer")
            if st == "ok":
                self._json(200, {"desc": ans})
            elif st == "stale":
                self._json(410, {"error": "stale session"})
            else:
                self._send(204)
            return
        if path == "/api/status":
            self._json(200, SIG.status())
            return
        if path == "/api/info":
            self._json(200, {"viewer_urls": VIEWER_URLS})
            return
        self._send(404, b"not found", "text/plain")

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            data = self._read_json()
        except Exception as e:  # noqa: BLE001
            self._json(400, {"error": str(e)})
            return
        if path == "/api/offer":
            SIG.touch("viewer")
            sid = SIG.put_offer(data["desc"])
            log("PICO 側から接続要求 (session %d, %s)" % (sid, self.client_address[0]))
            self._json(200, {"session": sid})
            return
        if path == "/api/answer":
            SIG.touch("sender")
            ok = SIG.put_answer(int(data["session"]), data["desc"])
            if ok:
                log("PC 側が応答 (session %s)" % data["session"])
            self._json(200 if ok else 410, {"ok": ok})
            return
        if path == "/api/log":
            role = str(data.get("role", "?"))[:16]
            SIG.touch(role)
            text = str(data.get("text", ""))[:2000]
            log("%s: %s" % (role, text))
            self._json(200, {"ok": True})
            return
        self._send(404, b"not found", "text/plain")


_seen_clients = set()


def note_client(ip, what):
    """端末ごとに最初の 1 回だけログに出す（PICO から届いているかの確認用）"""
    key = (ip, what)
    if key in _seen_clients:
        return
    _seen_clients.add(key)
    log("%s から %s" % (ip, what))


class QuietServer(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        # 自己署名の証明書を PICO がまだ受け入れていない間の TLS エラーなどは端末ごとに 1 行だけ
        err = sys.exc_info()[1]
        if isinstance(err, ssl.SSLError):
            note_client(client_address[0], "接続あり（証明書の確認待ち。PICO で「詳細設定」→「アクセスする」）")
            return
        if isinstance(err, (ConnectionError, TimeoutError)):
            return
        log("リクエスト処理のエラー (%s): %r" % (client_address[0], err))


def make_handler(index_page):
    return type("H_" + index_page.strip("/").replace(".", "_"), (Handler,), {"index_page": index_page})


# ---------------------------------------------------------------- network / cert helpers
def local_ipv4s():
    ips = []
    for probe in ("192.168.123.1", "10.255.255.255", "8.8.8.8"):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect((probe, 1))
            ips.append(s.getsockname()[0])
            s.close()
        except OSError:
            pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.append(info[4][0])
    except OSError:
        pass
    out = []
    for ip in ips:
        if ip.startswith("127.") or ip.startswith("169.254.") or ip == "0.0.0.0" or ip in out:
            continue
        out.append(ip)
    return out


def _gen_cert_cryptography(cert_path, key_path, ips):
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import ec
    from cryptography.x509.oid import NameOID

    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, u"pano-web (self-signed)")])
    san = [x509.DNSName(u"localhost")] + [x509.IPAddress(ipaddress.ip_address(ip)) for ip in ["127.0.0.1"] + ips]
    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (x509.CertificateBuilder()
            .subject_name(name).issuer_name(name)
            .public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now - datetime.timedelta(days=1))
            .not_valid_after(now + datetime.timedelta(days=825))
            .add_extension(x509.SubjectAlternativeName(san), critical=False)
            .sign(key, hashes.SHA256()))
    with open(key_path, "wb") as f:
        f.write(key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                  serialization.NoEncryption()))
    with open(cert_path, "wb") as f:
        f.write(cert.public_bytes(serialization.Encoding.PEM))


def _find_openssl():
    exe = shutil.which("openssl")
    if exe:
        return exe
    home = os.path.expanduser("~")
    for p in (os.path.join(sys.prefix, "Library", "bin", "openssl.exe"),  # conda / miniforge の Python
              os.path.join(home, "miniforge3", "Library", "bin", "openssl.exe"),
              os.path.join(home, "anaconda3", "Library", "bin", "openssl.exe"),
              r"C:\Program Files\Git\usr\bin\openssl.exe", r"C:\Program Files\Git\mingw64\bin\openssl.exe",
              r"C:\Program Files (x86)\Git\usr\bin\openssl.exe"):
        if os.path.exists(p):
            return p
    return None


def _gen_cert_openssl(exe, cert_path, key_path, ips):
    # 設定ファイルを自前で渡す（conda の openssl は環境を有効にしていないと openssl.cnf が見つからず失敗する）
    san = ",".join(["DNS:localhost", "IP:127.0.0.1"] + ["IP:%s" % ip for ip in ips])
    cnf_path = os.path.join(os.path.dirname(cert_path), "openssl_pano.cnf")
    with open(cnf_path, "w", encoding="ascii") as f:
        f.write("[req]\ndistinguished_name = dn\nprompt = no\nx509_extensions = ext\n"
                "[dn]\nCN = pano-web self-signed\n"
                "[ext]\nsubjectAltName = %s\n" % san)
    cmd = [exe, "req", "-x509", "-newkey", "ec", "-pkeyopt", "ec_paramgen_curve:prime256v1", "-nodes",
           "-keyout", key_path, "-out", cert_path, "-days", "825", "-config", cnf_path]
    env = dict(os.environ, OPENSSL_CONF=cnf_path)
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
    if r.returncode != 0:
        raise RuntimeError("openssl で証明書を作れませんでした: %s" % r.stderr.decode("utf-8", "replace").strip()[-400:])


def ensure_cert(cert_dir, ips):
    os.makedirs(cert_dir, exist_ok=True)
    cert_path = os.path.join(cert_dir, "cert.pem")
    key_path = os.path.join(cert_dir, "key.pem")
    if os.path.exists(cert_path) and os.path.exists(key_path):
        return cert_path, key_path
    log("自己署名の証明書を作ります: %s" % cert_dir)
    try:
        _gen_cert_cryptography(cert_path, key_path, ips)
        return cert_path, key_path
    except ImportError:
        pass
    exe = _find_openssl()
    if exe:
        try:
            _gen_cert_openssl(exe, cert_path, key_path, ips)
            return cert_path, key_path
        except Exception as e:  # noqa: BLE001
            log(str(e))
    sys.exit("証明書を作れません。`py -3 -m pip install cryptography`（または python -m pip install cryptography）"
             "をしてからもう一度起動してください。")


THREE_FILE = os.path.join(HERE, "three.module.min.js")
THREE_URLS = ("https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.min.js",
              "https://unpkg.com/three@0.160.0/build/three.module.min.js")


def ensure_three():
    """three.js (表示ページが使う 3D ライブラリ) が無ければ一度だけダウンロードして置く"""
    if os.path.exists(THREE_FILE) and os.path.getsize(THREE_FILE) > 500000:
        return
    import urllib.request
    for url in THREE_URLS:
        try:
            log("three.js をダウンロードします: %s" % url)
            with urllib.request.urlopen(url, timeout=20) as r:
                data = r.read()
            if len(data) > 500000 and data.lstrip().startswith(b"/**"):
                with open(THREE_FILE, "wb") as f:
                    f.write(data)
                log("three.js を置きました (%d KB)" % (len(data) // 1024))
                return
        except Exception as e:  # noqa: BLE001
            log("  失敗: %r" % e)
    log("three.js を置けませんでした。PICO がインターネットにつながっていれば CDN から読みます")


# ---------------------------------------------------------------- main
def serve(server, name):
    try:
        server.serve_forever(poll_interval=0.5)
    except Exception as e:  # noqa: BLE001
        log("%s が止まりました: %r" % (name, e))


def main():
    global _log_file
    ap = argparse.ArgumentParser(description="360度カメラ → PICO ブラウザ (WebXR) の中継サーバー")
    ap.add_argument("--https-port", type=int, default=8443, help="PICO が開くポート (https)")
    ap.add_argument("--http-port", type=int, default=8080, help="PC の送信ページのポート (http, localhost のみ)")
    ap.add_argument("--bind", default="0.0.0.0", help="https の待ち受けアドレス")
    ap.add_argument("--cert-dir", default=os.path.join(HERE, "certs"))
    ap.add_argument("--log-dir", default=os.path.join(HERE, "logs"))
    ap.add_argument("--open", action="store_true", help="起動後に PC のブラウザで送信ページを開く")
    args = ap.parse_args()

    os.makedirs(args.log_dir, exist_ok=True)
    log_path = os.path.join(args.log_dir, "pano_web_%s.log" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    _log_file = open(log_path, "a", encoding="utf-8")

    ips = local_ipv4s()
    ensure_three()
    cert_path, key_path = ensure_cert(args.cert_dir, ips)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(cert_path, key_path)
    https = QuietServer((args.bind, args.https_port), make_handler("/viewer.html"))
    # ハンドシェイクは各リクエストのスレッドで行う（受付スレッドが止まらないように）
    https.socket = ctx.wrap_socket(https.socket, server_side=True, do_handshake_on_connect=False)

    http = QuietServer(("127.0.0.1", args.http_port), make_handler("/sender.html"))

    threading.Thread(target=serve, args=(https, "https"), daemon=True).start()
    threading.Thread(target=serve, args=(http, "http"), daemon=True).start()

    sender_url = "http://localhost:%d/sender.html" % args.http_port
    log("ログ: %s" % log_path)
    log("PC の Chrome で開く（送信）: %s" % sender_url)
    VIEWER_URLS[:] = ["https://%s:%d/" % (ip, args.https_port) for ip in ips]
    if ips:
        for url in VIEWER_URLS:
            log("PICO のブラウザで開く（表示）: %s" % url)
    else:
        log("PC の IP が分かりませんでした。ipconfig で調べて https://<IP>:%d/ を PICO で開いてください" % args.https_port)
    log("止めるときは Ctrl+C")
    if args.open:
        import webbrowser
        webbrowser.open(sender_url)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log("終了します")
    finally:
        https.shutdown()
        http.shutdown()


if __name__ == "__main__":
    main()
