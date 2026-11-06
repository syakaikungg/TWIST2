// PICO 側: PC から WebRTC で受けた 2:1 の360度映像を球の内側に貼り、WebXR で見回す
// three.js は同じフォルダのものを使い、無ければ CDN から（PICO がインターネットにつながっている場合）
let THREE;
try {
  THREE = await import('./three.module.min.js');
} catch (_) {
  THREE = await import('https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.min.js');
}
import { getJSON, postJSON, serverLog, iceGatheringDone, sleep, videoStats, statsLine } from './common.js';

const $ = (id) => document.getElementById(id);
const params = new URLSearchParams(location.search);
const PREF_CODEC = (params.get('codec') || 'h264').toLowerCase(); // h264 | vp8 | vp9 | av1
const video = $('video');

function setStatus(text) { $('status').textContent = text; hud.status = text; }

// ---------------------------------------------------------------- 正面の向き（端末に覚えさせる）
let yawDeg = 0;
try { yawDeg = Number(localStorage.getItem('pano_yaw_deg')) || 0; } catch (_) { /* 使えなくても動く */ }
function saveYaw() { try { localStorage.setItem('pano_yaw_deg', String(Math.round(yawDeg))); } catch (_) { /* 同上 */ } }

// ---------------------------------------------------------------- three.js
const renderer = new THREE.WebGLRenderer({ antialias: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType('local');
renderer.xr.setFoveation(0); // 周辺の解像度を落とさない（球に貼るだけなので重くない）
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x101010);
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.05, 1000);
camera.rotation.order = 'YXZ';
scene.add(camera);

// 映像待ちの間に見える目印（向きが分かるように薄い格子）
const grid = new THREE.Mesh(new THREE.SphereGeometry(100, 24, 12),
  new THREE.MeshBasicMaterial({ color: 0x3a3a37, wireframe: true }));
scene.add(grid);

// 2:1 正距円筒を内側から見る球。画像の中心が正面 (-Z) に来るように -90° 回す
const videoTex = new THREE.VideoTexture(video);
videoTex.colorSpace = THREE.SRGBColorSpace;
videoTex.minFilter = THREE.LinearFilter;
videoTex.magFilter = THREE.LinearFilter;
videoTex.generateMipmaps = false;
const sphereGeo = new THREE.SphereGeometry(100, 128, 64);
sphereGeo.scale(-1, 1, 1);
const sphere = new THREE.Mesh(sphereGeo, new THREE.MeshBasicMaterial({ map: videoTex }));
sphere.visible = false;
scene.add(sphere);
function applyYaw() { sphere.rotation.y = -Math.PI / 2 + THREE.MathUtils.degToRad(yawDeg); }
applyYaw();

// VR 中に目の前の下に出す情報板（カメラの子にすると頭についてくる）
const hud = { status: '', stats: '', warn: '', visible: true, dirty: true };
const hudCanvas = document.createElement('canvas');
hudCanvas.width = 1024;
hudCanvas.height = 320;
const hudTex = new THREE.CanvasTexture(hudCanvas);
hudTex.colorSpace = THREE.SRGBColorSpace;
const hudMesh = new THREE.Mesh(new THREE.PlaneGeometry(0.6, 0.1875),
  new THREE.MeshBasicMaterial({ map: hudTex, transparent: true, depthTest: false, depthWrite: false }));
hudMesh.position.set(0, -0.22, -0.8);
hudMesh.renderOrder = 999;
camera.add(hudMesh);

function drawHud() {
  if (!hud.dirty) return;
  hud.dirty = false;
  const g = hudCanvas.getContext('2d');
  g.clearRect(0, 0, hudCanvas.width, hudCanvas.height);
  g.fillStyle = 'rgba(16,16,16,0.72)';
  g.beginPath();
  g.roundRect ? g.roundRect(0, 0, 1024, 320, 28) : g.rect(0, 0, 1024, 320);
  g.fill();
  g.fillStyle = '#ececea';
  g.font = '600 40px system-ui, sans-serif';
  g.fillText(hud.status.slice(0, 24), 36, 62);
  g.font = '30px system-ui, sans-serif';
  g.fillStyle = '#c8c8c4';
  g.fillText(hud.stats.slice(0, 60), 36, 116);
  g.fillText(`正面 ${Math.round(yawDeg)}°（スティック左右で回す）`, 36, 164);
  g.fillStyle = '#ececea';
  g.fillText('A・X: VRを終わる　B・Y: 正面を戻す　トリガー: この表示', 36, 212);
  if (hud.warn) { g.fillStyle = '#f6ad55'; g.fillText(hud.warn.slice(0, 32), 36, 270); }
  hudTex.needsUpdate = true;
}

// ---------------------------------------------------------------- 画面での見回し（VR の外）
let lon = 0, lat = 0, drag = null;
renderer.domElement.addEventListener('pointerdown', (e) => { drag = { x: e.clientX, y: e.clientY, lon, lat }; renderer.domElement.setPointerCapture(e.pointerId); });
renderer.domElement.addEventListener('pointermove', (e) => {
  if (!drag) return;
  lon = drag.lon + (e.clientX - drag.x) * 0.15;
  lat = Math.max(-85, Math.min(85, drag.lat + (e.clientY - drag.y) * 0.15));
});
renderer.domElement.addEventListener('pointerup', () => { drag = null; });
renderer.domElement.addEventListener('wheel', (e) => {
  camera.fov = Math.max(30, Math.min(100, camera.fov + Math.sign(e.deltaY) * 4));
  camera.updateProjectionMatrix();
}, { passive: true });
window.addEventListener('keydown', (e) => {
  if (e.key === 'ArrowLeft') { yawDeg -= 5; applyYaw(); saveYaw(); hud.dirty = true; }
  if (e.key === 'ArrowRight') { yawDeg += 5; applyYaw(); saveYaw(); hud.dirty = true; }
});
$('yawL').addEventListener('click', () => { yawDeg -= 15; applyYaw(); saveYaw(); hud.dirty = true; });
$('yawR').addEventListener('click', () => { yawDeg += 15; applyYaw(); saveYaw(); hud.dirty = true; });
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// ---------------------------------------------------------------- コントローラー
for (const i of [0, 1]) {
  const c = renderer.xr.getController(i);
  c.addEventListener('select', () => { hud.visible = !hud.visible; });
  scene.add(c);
}
const prevButtons = new Map();
function pollGamepads(dt) {
  const session = renderer.xr.getSession();
  if (!session) return;
  for (const src of session.inputSources) {
    const gp = src.gamepad;
    if (!gp) continue;
    const a2 = gp.axes[2] || 0, a0 = gp.axes[0] || 0;
    const ax = Math.abs(a2) >= Math.abs(a0) ? a2 : a0;
    if (Math.abs(ax) > 0.25) { yawDeg += ax * 60 * dt; applyYaw(); hud.dirty = true; }
    const key = src.handedness;
    const b4 = !!(gp.buttons[4] && gp.buttons[4].pressed); // A / X
    const b5 = !!(gp.buttons[5] && gp.buttons[5].pressed); // B / Y
    const prev = prevButtons.get(key) || {};
    if (b5 && !prev.b5) { yawDeg = 0; applyYaw(); saveYaw(); hud.dirty = true; }
    if (b4 && !prev.b4) { saveYaw(); session.end().catch(() => {}); }
    prevButtons.set(key, { b4, b5 });
  }
}

let lastT = 0;
let yawSaveAt = 0;
renderer.setAnimationLoop((t) => {
  const dt = lastT ? Math.min(0.1, (t - lastT) / 1000) : 0;
  lastT = t;
  if (renderer.xr.isPresenting) {
    pollGamepads(dt);
    hudMesh.visible = hud.visible;
    drawHud();
    if (t > yawSaveAt) { saveYaw(); yawSaveAt = t + 2000; }
  } else {
    hudMesh.visible = false;
    camera.rotation.y = THREE.MathUtils.degToRad(lon);
    camera.rotation.x = THREE.MathUtils.degToRad(lat);
  }
  renderer.render(scene, camera);
});

// ---------------------------------------------------------------- VR に入る
async function setupVrButton() {
  const btn = $('vr');
  if (!window.isSecureContext) {
    $('warn').textContent = 'https で開いていないので VR に入れません。https://<PCのIP>:8443/ で開いてください。';
    return;
  }
  if (!navigator.xr) { btn.textContent = 'このブラウザは VR 非対応'; return; }
  let ok = false;
  try { ok = await navigator.xr.isSessionSupported('immersive-vr'); } catch (_) { ok = false; }
  if (!ok) { btn.textContent = 'VR 非対応（画面で見回しのみ）'; return; }
  btn.disabled = false;
  btn.addEventListener('click', async () => {
    video.play().catch(() => {});
    try {
      const s = await navigator.xr.requestSession('immersive-vr', { optionalFeatures: ['local-floor'] });
      await renderer.xr.setSession(s);
      hud.dirty = true;
      serverLog('viewer', 'VR に入りました');
      s.addEventListener('end', () => serverLog('viewer', 'VR を出ました'));
    } catch (e) {
      $('warn').textContent = 'VR に入れません: ' + e.message;
    }
  });
}

// ---------------------------------------------------------------- WebRTC（受け側）
let pc = null;
let mySession = 0;
let connecting = false;
let prevStats = null;
let takenOver = false;

function preferCodec(transceiver, name) {
  if (!transceiver.setCodecPreferences || !window.RTCRtpReceiver || !RTCRtpReceiver.getCapabilities) return;
  const caps = RTCRtpReceiver.getCapabilities('video').codecs;
  const want = ('video/' + name).toLowerCase();
  const first = caps.filter((c) => c.mimeType.toLowerCase() === want);
  const score = (c) => {
    const f = c.sdpFmtpLine || '';
    return (f.includes('packetization-mode=1') ? 0 : 2) + (f.includes('profile-level-id=42e0') ? 0 : 1);
  };
  first.sort((a, b) => score(a) - score(b));
  if (!first.length) return;
  transceiver.setCodecPreferences([...first, ...caps.filter((c) => !first.includes(c))]);
}

async function connect(reason) {
  if (connecting) return;
  connecting = true;
  takenOver = false;
  let retry = false;
  try {
    if (pc) { pc.close(); pc = null; }
    prevStats = null;
    setStatus('PC に接続しています…');
    const conn = new RTCPeerConnection({ iceServers: [] });
    pc = conn;
    const tr = conn.addTransceiver('video', { direction: 'recvonly' });
    preferCodec(tr, PREF_CODEC);
    conn.ontrack = (e) => {
      // 低遅延寄りに（受信側のバッファを最小に）
      try { e.receiver.playoutDelayHint = 0; } catch (_) { /* 非対応 */ }
      try { if ('jitterBufferTarget' in e.receiver) e.receiver.jitterBufferTarget = 0; } catch (_) { /* 非対応 */ }
      video.srcObject = e.streams[0] || new MediaStream([e.track]);
      video.play().catch(() => {});
    };
    conn.onconnectionstatechange = () => onConnState(conn);
    await conn.setLocalDescription(await conn.createOffer());
    await iceGatheringDone(conn);
    const r = await postJSON('/api/offer', { desc: { type: conn.localDescription.type, sdp: conn.localDescription.sdp } });
    if (r.status !== 200) throw new Error('サーバーが offer を受け付けません (' + r.status + ')');
    mySession = r.body.session;
    serverLog('viewer', `接続を始めます (session ${mySession}, ${reason || '起動'}, codec優先 ${PREF_CODEC})`);
    let waited = 0;
    for (;;) {
      if (conn !== pc) return;
      const a = await getJSON(`/api/answer?session=${mySession}`);
      if (a.status === 200 && a.body) { await conn.setRemoteDescription(a.body.desc); break; }
      if (a.status === 410) { takenOver = true; setStatus('別の端末が表示を始めたので止めました。「再接続」で取り戻せます'); conn.close(); pc = null; return; }
      waited += 10;
      setStatus(`PC の送信ページの応答を待っています（${waited}秒）。PC でカメラを開いてください`);
    }
    setStatus('つないでいます…');
  } catch (e) {
    setStatus('接続できません: ' + e.message + '（3秒後にやり直します）');
    retry = true;
  } finally {
    connecting = false;
  }
  if (retry) setTimeout(() => connect('エラー後の再試行'), 3000);
}

let discTimer = null;
async function onConnState(conn) {
  if (conn !== pc) return;
  const s = conn.connectionState;
  serverLog('viewer', `接続状態 ${s} (session ${mySession})`);
  clearTimeout(discTimer);
  if (s === 'connected') { setStatus('受信中'); return; }
  if (s === 'disconnected') { setStatus('通信が途切れています…'); discTimer = setTimeout(() => maybeReconnect(conn), 4000); return; }
  if (s === 'failed' || s === 'closed') maybeReconnect(conn);
}

async function maybeReconnect(conn) {
  if (conn !== pc || takenOver) return;
  try {
    const st = await getJSON('/api/status');
    if (st.body && st.body.session !== mySession) {
      takenOver = true;
      setStatus('別の端末が表示を始めたので止めました。「再接続」で取り戻せます');
      conn.close(); pc = null;
      return;
    }
  } catch (_) { /* サーバーが落ちていても再接続を試す */ }
  setStatus('切れました。つなぎ直します…');
  await sleep(1000);
  connect('切断後の再接続');
}

$('reconnect').addEventListener('click', () => connect('再接続ボタン'));

video.addEventListener('resize', () => {
  const w = video.videoWidth, h = video.videoHeight;
  if (!w || !h) return;
  sphere.visible = true;
  grid.visible = false;
  const off = Math.abs(w / h - 2) > 0.02;
  hud.warn = off ? `2:1 ではありません（${w}x${h}）。歪んで見えます` : '';
  $('warn').textContent = hud.warn;
  hud.dirty = true;
});

// 1 秒ごとに受信の様子を表示、5 秒ごとに PC のログへ。映像が 4 秒止まったらつなぎ直す
let tick = 0;
let stallSec = 0;
setInterval(async () => {
  if (!pc || connecting) { stallSec = 0; return; }
  try {
    const conn = pc;
    const s = await videoStats(conn, 'inbound-rtp', prevStats);
    const moved = s && prevStats && s.frames > prevStats.frames;
    const started = s && s.frames > 0;
    prevStats = s;
    stallSec = started && !moved ? stallSec + 1 : 0;
    if (stallSec >= 4) {
      stallSec = 0;
      serverLog('viewer', '映像が 4 秒止まったのでつなぎ直します');
      maybeReconnect(conn);
      return;
    }
    const line = statsLine(s);
    $('stats').textContent = '受信: ' + line;
    if (hud.stats !== line) { hud.stats = line; hud.dirty = true; }
    if (++tick % 5 === 0 && s) serverLog('viewer', line + (renderer.xr.isPresenting ? ' / VR中' : ''));
  } catch (_) { /* 次で */ }
}, 1000);

setupVrButton();
connect();
