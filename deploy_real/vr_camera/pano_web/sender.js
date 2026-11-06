// PC 側: カメラを開き、PICO（viewer）からの接続要求に WebRTC で映像を返す
import { getJSON, postJSON, serverLog, iceGatheringDone, sleep, videoStats, statsLine } from './common.js';

const $ = (id) => document.getElementById(id);
const params = new URLSearchParams(location.search);

let stream = null;
let pc = null;
let lastSession = 0;
let prevStats = null;

function setStatus(text, cls = '') {
  $('status').textContent = text;
  $('status').className = cls;
}

// ---------------------------------------------------------------- camera
async function listCameras() {
  let devs = (await navigator.mediaDevices.enumerateDevices()).filter((d) => d.kind === 'videoinput');
  if (devs.length && !devs[0].label) {
    // 名前は許可の後でないと出ないので、一度だけ許可をもらう
    try {
      const s = await navigator.mediaDevices.getUserMedia({ video: true });
      s.getTracks().forEach((t) => t.stop());
      devs = (await navigator.mediaDevices.enumerateDevices()).filter((d) => d.kind === 'videoinput');
    } catch (e) {
      setStatus('カメラの許可がありません: ' + e.message, 'bad');
    }
  }
  const sel = $('camera');
  const keep = sel.value;
  sel.innerHTML = '';
  devs.forEach((d, i) => {
    const o = document.createElement('option');
    o.value = d.deviceId;
    o.textContent = d.label || `カメラ ${i + 1}`;
    sel.appendChild(o);
  });
  const pick = devs.find((d) => d.deviceId === keep) || devs.find((d) => /insta360|360|theta/i.test(d.label));
  if (pick) sel.value = pick.deviceId;
  if (!devs.length) setStatus('カメラが見つかりません。Insta360 をウェブカメラモードでつないでください', 'bad');
}

async function openCamera() {
  const fps = Number($('fps').value) || 30;
  const video = { deviceId: $('camera').value ? { exact: $('camera').value } : undefined, frameRate: { ideal: fps } };
  if ($('res').value !== 'default') {
    const [w, h] = $('res').value.split('x').map(Number);
    video.width = { ideal: w };
    video.height = { ideal: h };
  }
  const s = await navigator.mediaDevices.getUserMedia({ video, audio: false });
  if (stream) stream.getTracks().forEach((t) => t.stop());
  stream = s;
  const track = stream.getVideoTracks()[0];
  track.contentHint = 'detail';
  $('preview').srcObject = stream;
  const st = track.getSettings();
  const ratio = st.width && st.height ? st.width / st.height : 0;
  const ok = Math.abs(ratio - 2) < 0.02;
  $('camInfo').innerHTML = `開いた映像: <b>${st.width}x${st.height}</b> ${Math.round(st.frameRate || 0)}fps（${track.label}）` +
    (ok ? ' <span class="ok">2:1 OK</span>' : ' <span class="warn">2:1 ではありません。PICO では歪んで見えます</span>');
  serverLog('sender', `カメラ ${track.label} ${st.width}x${st.height} ${Math.round(st.frameRate || 0)}fps`);
  // すでにつながっていれば映像だけ差し替える
  if (pc) {
    const snd = pc.getSenders().find((x) => x.track && x.track.kind === 'video');
    if (snd) await snd.replaceTrack(track);
  }
  setStatus(pc ? '送信中' : 'PICO の接続を待っています', pc ? 'ok' : '');
}

// ---------------------------------------------------------------- signaling
// Chrome 固有: 受け取った offer の映像コーデックに開始・最小・最大ビットレートを書き足す。
// 送る側は相手の記述からこれを読むので、ここで足すと最初から高い画質で始まる。
function withBitrateHints(sdp, maxKbps) {
  const startKbps = Math.min(maxKbps, Math.max(2000, Math.round(maxKbps * 0.6)));
  const minKbps = Math.min(startKbps, 1500);
  const hint = `x-google-start-bitrate=${startKbps};x-google-min-bitrate=${minKbps};x-google-max-bitrate=${maxKbps}`;
  const lines = sdp.split('\r\n');
  const pts = new Set();
  lines.forEach((l) => {
    const m = l.match(/^a=rtpmap:(\d+) (H264|VP8|VP9|AV1)\/90000/i);
    if (m) pts.add(m[1]);
  });
  const out = [];
  const hasFmtp = new Set();
  lines.forEach((l) => {
    const m = l.match(/^a=fmtp:(\d+) (.*)$/);
    if (m && pts.has(m[1])) {
      hasFmtp.add(m[1]);
      out.push(`${l};${hint}`);
    } else {
      out.push(l);
    }
  });
  const final = [];
  out.forEach((l) => {
    final.push(l);
    const m = l.match(/^a=rtpmap:(\d+) /);
    if (m && pts.has(m[1]) && !hasFmtp.has(m[1])) final.push(`a=fmtp:${m[1]} ${hint}`);
  });
  return final.join('\r\n');
}

async function answerOffer(session, desc) {
  if (pc) { pc.close(); pc = null; }
  const maxKbps = Math.round((Number($('mbps').value) || 15) * 1000);
  const fps = Number($('fps').value) || 30;
  const conn = new RTCPeerConnection({ iceServers: [] });
  pc = conn;
  prevStats = null;
  conn.onconnectionstatechange = () => {
    const s = conn.connectionState;
    serverLog('sender', `接続状態 ${s} (session ${session})`);
    if (conn !== pc) return;
    if (s === 'connected') setStatus('送信中', 'ok');
    else if (s === 'failed' || s === 'closed') { setStatus('切れました。PICO の再接続を待っています', 'warn'); pc = null; conn.close(); }
    else if (s === 'disconnected') setStatus('通信が途切れています…', 'warn');
  };
  await conn.setRemoteDescription({ type: desc.type, sdp: withBitrateHints(desc.sdp, maxKbps) });
  const track = stream.getVideoTracks()[0];
  const snd = conn.addTrack(track, stream);
  const answer = await conn.createAnswer();
  await conn.setLocalDescription(answer);
  try {
    const p = snd.getParameters();
    if (!p.encodings || !p.encodings.length) p.encodings = [{}];
    p.encodings[0].maxBitrate = maxKbps * 1000;
    p.encodings[0].maxFramerate = fps;
    await snd.setParameters(p);
  } catch (e) {
    serverLog('sender', 'ビットレート上限の設定に失敗: ' + e.message);
  }
  try {
    // 回線や CPU が苦しいときは fps を落としてでも解像度を保つ（360度は細かさが大事）。古い Chrome は無視する
    const p = snd.getParameters();
    p.degradationPreference = 'maintain-resolution';
    await snd.setParameters(p);
  } catch (_) { /* 対応していなければ contentHint=detail で代用 */ }
  await iceGatheringDone(conn);
  const r = await postJSON('/api/answer', { session, desc: { type: conn.localDescription.type, sdp: conn.localDescription.sdp } });
  if (r.status !== 200) serverLog('sender', `answer が古い (session ${session})`);
  setStatus('PICO とつないでいます…');
}

async function offerLoop() {
  for (;;) {
    try {
      const r = await getJSON(`/api/offer?after=${lastSession}`);
      if (r.status === 200 && r.body) {
        if (!stream) {
          setStatus('PICO から接続要求あり。先に「カメラを開く」を押してください', 'warn');
          await sleep(1000);
          continue;
        }
        lastSession = r.body.session;
        await answerOffer(r.body.session, r.body.desc);
      }
    } catch (e) {
      setStatus('サーバーとつながりません（pano_server.py は動いていますか）: ' + e.message, 'bad');
      await sleep(2000);
    }
  }
}

// ---------------------------------------------------------------- stats / clock
let statTick = 0;
setInterval(async () => {
  if (!pc) { $('stats').textContent = '-'; return; }
  try {
    const s = await videoStats(pc, 'outbound-rtp', prevStats);
    prevStats = s;
    $('stats').textContent = '送信: ' + statsLine(s);
    if (++statTick % 5 === 0 && s) serverLog('sender', statsLine(s));
  } catch (_) { /* 次で */ }
}, 1000);

function tickClock() {
  if ($('showClock').checked) {
    const d = new Date();
    const mm = String(d.getMinutes()).padStart(2, '0');
    const ss = String(d.getSeconds()).padStart(2, '0');
    const ms = String(d.getMilliseconds()).padStart(3, '0');
    $('clock').textContent = `${mm}:${ss}.${ms}`;
  }
  requestAnimationFrame(tickClock);
}
$('showClock').addEventListener('change', () => { $('clock').style.display = $('showClock').checked ? 'block' : 'none'; });
requestAnimationFrame(tickClock);

// ---------------------------------------------------------------- start
$('start').addEventListener('click', () => openCamera().catch((e) => setStatus('カメラを開けません: ' + e.message, 'bad')));
navigator.mediaDevices.addEventListener('devicechange', () => listCameras());

(async () => {
  try {
    const info = await getJSON('/api/info');
    if (info.body && info.body.viewer_urls) {
      $('viewerUrl').textContent = 'PICO のブラウザで開く: ' + (info.body.viewer_urls.join(' または ') || '（サーバーの画面を見てください）');
    }
  } catch (_) { /* なくても動く */ }
  await listCameras();
  setStatus('「カメラを開く」を押してください');
  if (params.get('autostart') === '1') await openCamera().catch((e) => setStatus('カメラを開けません: ' + e.message, 'bad'));
  offerLoop();
})();
