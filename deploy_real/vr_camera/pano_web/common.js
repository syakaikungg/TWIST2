// sender.js / viewer.js 共通の小物

export async function postJSON(path, obj) {
  const r = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(obj),
  });
  let body = null;
  try { body = await r.json(); } catch (_) { /* 空の応答 */ }
  return { status: r.status, body };
}

// サーバーの long-poll (最大 10 秒待って 204) を 1 回だけ叩く
export async function getJSON(path) {
  const r = await fetch(path, { cache: 'no-store' });
  if (r.status === 204) return { status: 204, body: null };
  let body = null;
  try { body = await r.json(); } catch (_) { /* 空の応答 */ }
  return { status: r.status, body };
}

// サーバーのターミナルとログファイルに 1 行出す
export function serverLog(role, text) {
  postJSON('/api/log', { role, text }).catch(() => {});
}

// ICE の候補集めが終わる (または timeoutMs 経過) まで待つ。LAN なので候補はすぐ揃う
export function iceGatheringDone(pc, timeoutMs = 2000) {
  if (pc.iceGatheringState === 'complete') return Promise.resolve();
  return new Promise((resolve) => {
    const t = setTimeout(done, timeoutMs);
    function done() {
      clearTimeout(t);
      pc.removeEventListener('icegatheringstatechange', check);
      resolve();
    }
    function check() { if (pc.iceGatheringState === 'complete') done(); }
    pc.addEventListener('icegatheringstatechange', check);
  });
}

export const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// getStats() から映像の要点を取り出す。prev は前回の戻り値（ビットレート・fps 計算用）
export async function videoStats(pc, kind, prev) {
  const report = await pc.getStats();
  let rtp = null;
  let pair = null;
  const codecs = {};
  report.forEach((s) => {
    if (s.type === kind && s.kind === 'video') rtp = s;
    if (s.type === 'codec') codecs[s.id] = s;
    if (s.type === 'candidate-pair' && s.nominated && s.state === 'succeeded') pair = s;
  });
  if (!rtp) return null;
  const now = rtp.timestamp;
  const bytes = kind === 'inbound-rtp' ? rtp.bytesReceived : rtp.bytesSent;
  const frames = kind === 'inbound-rtp' ? rtp.framesDecoded : rtp.framesEncoded;
  const out = {
    t: now,
    bytes,
    frames,
    width: rtp.frameWidth || 0,
    height: rtp.frameHeight || 0,
    fps: rtp.framesPerSecond || 0,
    codec: (codecs[rtp.codecId] && codecs[rtp.codecId].mimeType || '').replace('video/', ''),
    impl: kind === 'inbound-rtp' ? (rtp.decoderImplementation || '') : (rtp.encoderImplementation || ''),
    rttMs: pair && pair.currentRoundTripTime != null ? pair.currentRoundTripTime * 1000 : null,
    mbps: 0,
    jitterBufMs: null,
    dropped: rtp.framesDropped || 0,
    lost: rtp.packetsLost || 0,
    limit: rtp.qualityLimitationReason || '',
    jbDelay: rtp.jitterBufferDelay || 0,
    jbCount: rtp.jitterBufferEmittedCount || 0,
  };
  if (prev && now > prev.t) {
    const dt = (now - prev.t) / 1000;
    out.mbps = ((bytes - prev.bytes) * 8) / dt / 1e6;
    if (!out.fps) out.fps = (frames - prev.frames) / dt;
    const dc = out.jbCount - prev.jbCount;
    if (dc > 0) out.jitterBufMs = ((out.jbDelay - prev.jbDelay) / dc) * 1000;
  }
  return out;
}

export function statsLine(s) {
  if (!s) return '映像なし';
  const parts = [
    `${s.width}x${s.height}`,
    `${s.fps.toFixed(0)}fps`,
    `${s.mbps.toFixed(1)}Mbps`,
    s.codec,
  ];
  if (s.impl) parts.push(s.impl);
  if (s.jitterBufMs != null) parts.push(`受信待ち${s.jitterBufMs.toFixed(0)}ms`);
  if (s.rttMs != null) parts.push(`RTT${s.rttMs.toFixed(0)}ms`);
  if (s.limit && s.limit !== 'none') parts.push(`制限:${s.limit}`);
  if (s.lost) parts.push(`欠落${s.lost}`);
  return parts.join(' / ');
}
