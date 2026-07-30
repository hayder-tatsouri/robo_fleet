// Robo_Fleet - Novation City live dashboard
// Renders the actual occupancy grid + robots + paths on a pannable/zoomable canvas.

const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
const robotsDiv = document.getElementById('robots');
const statusEl = document.getElementById('status');
const viewInfoEl = document.getElementById('viewinfo');

// --- Map metadata (mirrors novation_city.yaml) ---
const MAP = {
  resolution: 1.0,       // m/pixel
  origin_x: -600.0,      // world x of pixel (0, H-1) — YAML origin is bottom-left
  origin_y: -600.0,
  width: 1200,           // pixels
  height: 1200,
  img: null,             // Image element (loaded async)
  ready: false,
};

// --- View state (world coords centred at (cx,cy), spanning `spanX` metres horizontally) ---
const view = {
  cx: 0.0,               // centre in world coords
  cy: 0.0,
  span: 400.0,           // horizontal metres visible (aspect follows canvas)
  follow: false,
};

let robots = [];
let trails = {};         // robot_id -> [{x,y}]
let plans = {};          // robot_id -> [{x,y}]  latest planned path

// --- Map image loader ---
const mapImg = new Image();
mapImg.onload = () => {
  MAP.img = mapImg;
  MAP.ready = true;
  render();
};
mapImg.onerror = () => {
  console.warn('map image failed to load, will render without background');
};
mapImg.src = 'novation_city_color.png';

// --- WebSocket ---
const WS_URL = new URLSearchParams(window.location.search).get('ws') || 'ws://localhost:8080';
let ws = null;

function connect() {
  ws = new WebSocket(WS_URL);
  ws.onopen = () => { statusEl.textContent = 'Connected'; statusEl.className = 'connected'; };
  ws.onclose = () => { statusEl.textContent = 'Disconnected'; statusEl.className = 'disconnected'; setTimeout(connect, 2000); };
  ws.onmessage = handleWsMessage;
}

function handleWsMessage(e) {
  const data = JSON.parse(e.data);
  if (data.type === 'fleet_state') {
    robots = data.robots;
    updateTrails();
    if (view.follow && robots.length) {
      view.cx = robots[0].x;
      view.cy = robots[0].y;
    }
    render();
    updateSidebar();
  } else if (data.type === 'plan') {
    // { type: 'plan', robot_id: 'pguard', poses: [{x,y}, ...] }
    plans[data.robot_id] = data.poses || [];
    render();
  } else if (data.type === 'chat_step') {
    const typing = document.getElementById('typing-indicator');
    if (data.status === 'running') {
      if (typing) typing.textContent = data.agent + ' agent running...';
    } else if (data.status === 'done') {
      if (typing) typing.textContent = 'Processing...';
    }
  } else if (data.type === 'chat_response') {
    const typing = document.getElementById('typing-indicator');
    if (typing) typing.remove();
    addChatMessage(data.message, 'bot');
    if (data.tool_used) addChatMessage('\u26a1 ' + data.tool_used, 'tool');
  }
}

function updateTrails() {
  robots.forEach(r => {
    if (!trails[r.id]) trails[r.id] = [];
    const last = trails[r.id][trails[r.id].length - 1];
    if (!last || Math.hypot(last.x - r.x, last.y - r.y) > 0.3) {
      trails[r.id].push({x: r.x, y: r.y});
      if (trails[r.id].length > 500) trails[r.id].shift();
    }
  });
}

// --- Coordinate helpers ---
// world (x,y) -> canvas pixel (px, py)
function worldToCanvas(x, y) {
  const w = canvas.width, h = canvas.height;
  const aspect = h / w;
  const spanY = view.span * aspect;
  const px = ((x - (view.cx - view.span / 2)) / view.span) * w;
  const py = (((view.cy + spanY / 2) - y) / spanY) * h;
  return [px, py];
}

function canvasToWorld(px, py) {
  const w = canvas.width, h = canvas.height;
  const aspect = h / w;
  const spanY = view.span * aspect;
  const x = (px / w) * view.span + (view.cx - view.span / 2);
  const y = (view.cy + spanY / 2) - (py / h) * spanY;
  return [x, y];
}

// --- Rendering ---
function render() {
  const parent = canvas.parentElement;
  if (canvas.width !== parent.clientWidth || canvas.height !== parent.clientHeight) {
    canvas.width = parent.clientWidth;
    canvas.height = parent.clientHeight;
  }
  const W = canvas.width, H = canvas.height;

  ctx.fillStyle = '#0a0a1a';
  ctx.fillRect(0, 0, W, H);

  drawMap();
  drawGrid();
  drawOrigin();
  drawPlans();
  drawTrails();
  drawRobots();
  updateViewInfo();
}

function drawMap() {
  if (!MAP.ready) return;
  // Map image spans world x from origin_x to origin_x + width*res
  //                     world y from origin_y to origin_y + height*res
  // PGM row 0 is top of image but corresponds to world y_max (image y-flipped vs world).
  const wxMin = MAP.origin_x;
  const wyMax = MAP.origin_y + MAP.height * MAP.resolution;
  const [px0, py0] = worldToCanvas(wxMin, wyMax);          // top-left of image
  const [px1, py1] = worldToCanvas(wxMin + MAP.width * MAP.resolution,
                                    MAP.origin_y);         // bottom-right
  const dw = px1 - px0;
  const dh = py1 - py0;
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(MAP.img, px0, py0, dw, dh);
}

function drawGrid() {
  const step = view.span > 200 ? 50 : view.span > 50 ? 10 : 5;
  const [xL, yT] = canvasToWorld(0, 0);
  const [xR, yB] = canvasToWorld(canvas.width, canvas.height);
  ctx.strokeStyle = 'rgba(255,255,255,0.06)';
  ctx.lineWidth = 1;
  const x0 = Math.floor(xL / step) * step;
  for (let x = x0; x <= xR; x += step) {
    const [px, py] = worldToCanvas(x, yT);
    const [, py2] = worldToCanvas(x, yB);
    ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px, py2); ctx.stroke();
  }
  const y0 = Math.floor(yB / step) * step;
  for (let y = y0; y <= yT; y += step) {
    const [px, py] = worldToCanvas(xL, y);
    const [px2] = worldToCanvas(xR, y);
    ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px2, py); ctx.stroke();
  }
}

function drawOrigin() {
  const [ox, oy] = worldToCanvas(0, 0);
  ctx.strokeStyle = '#f87171';
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(ox - 8, oy); ctx.lineTo(ox + 8, oy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(ox, oy - 8); ctx.lineTo(ox, oy + 8); ctx.stroke();
}

function drawPlans() {
  Object.entries(plans).forEach(([id, poses]) => {
    if (!poses || poses.length < 2) return;
    ctx.strokeStyle = '#4ade80';
    ctx.lineWidth = 2;
    ctx.beginPath();
    const [sx, sy] = worldToCanvas(poses[0].x, poses[0].y);
    ctx.moveTo(sx, sy);
    for (let i = 1; i < poses.length; i++) {
      const [px, py] = worldToCanvas(poses[i].x, poses[i].y);
      ctx.lineTo(px, py);
    }
    ctx.stroke();
  });
}

function drawTrails() {
  const colors = ['#8b5cf6', '#06b6d4', '#f59e0b', '#ec4899', '#10b981'];
  robots.forEach((r, idx) => {
    const trail = trails[r.id] || [];
    if (trail.length < 2) return;
    ctx.strokeStyle = colors[idx % colors.length] + '80';
    ctx.lineWidth = 2;
    ctx.beginPath();
    const [sx, sy] = worldToCanvas(trail[0].x, trail[0].y);
    ctx.moveTo(sx, sy);
    trail.forEach(p => { const [px, py] = worldToCanvas(p.x, p.y); ctx.lineTo(px, py); });
    ctx.stroke();
  });
}

function drawRobots() {
  const colors = ['#8b5cf6', '#06b6d4', '#f59e0b', '#ec4899', '#10b981'];
  // Robot chassis is 1.5x1.1 m — scale on canvas
  robots.forEach((r, idx) => {
    const [rx, ry] = worldToCanvas(r.x, r.y);
    const color = colors[idx % colors.length];
    const [x2] = worldToCanvas(r.x + 0.75, r.y);   // half length
    const halfLen = Math.max(4, Math.abs(x2 - rx));
    const [, y2] = worldToCanvas(r.x, r.y + 0.55);
    const halfWid = Math.max(3, Math.abs(y2 - ry));

    // Goal line + goal marker
    if (r.goal) {
      const [gx, gy] = worldToCanvas(r.goal.x, r.goal.y);
      ctx.strokeStyle = '#60a5fa';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 4]);
      ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(gx, gy); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = 'rgba(96,165,250,0.35)';
      ctx.strokeStyle = '#60a5fa';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(gx, gy, 9, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      ctx.fillStyle = '#60a5fa';
      ctx.beginPath(); ctx.arc(gx, gy, 3, 0, Math.PI * 2); ctx.fill();
    }

    // Chassis as rotated rectangle
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(-r.theta);
    ctx.fillStyle = r.online ? color : '#4b5563';
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.rect(-halfLen, -halfWid, halfLen * 2, halfWid * 2);
    ctx.fill(); ctx.stroke();
    // Heading indicator: bright triangle at front
    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.moveTo(halfLen, 0);
    ctx.lineTo(halfLen - 6, -halfWid * 0.6);
    ctx.lineTo(halfLen - 6, halfWid * 0.6);
    ctx.closePath(); ctx.fill();
    ctx.restore();

    // Highlight ring so we always see it even when far zoomed out
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(rx, ry, Math.max(halfLen, halfWid) + 4, 0, Math.PI * 2); ctx.stroke();

    // Label
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 11px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(r.id, rx, ry - (Math.max(halfLen, halfWid) + 10));
  });
}

function updateViewInfo() {
  const [xL, yT] = canvasToWorld(0, 0);
  const [xR, yB] = canvasToWorld(canvas.width, canvas.height);
  viewInfoEl.textContent =
    `view x:[${xL.toFixed(0)}, ${xR.toFixed(0)}]  y:[${yB.toFixed(0)}, ${yT.toFixed(0)}]  span:${view.span.toFixed(0)}m  centre:(${view.cx.toFixed(1)}, ${view.cy.toFixed(1)})`;
}

function updateSidebar() {
  const colors = ['#8b5cf6', '#06b6d4', '#f59e0b', '#ec4899', '#10b981'];
  robotsDiv.innerHTML = robots.map((r, i) => {
    const batColor = r.battery > 50 ? '#4ade80' : r.battery > 20 ? '#fbbf24' : '#f87171';
    const statusClass = r.online ? (r.status === 'navigating' ? 'status-navigating' : 'status-idle') : 'status-offline';
    let goalHtml = '';
    if (r.goal) {
      const d = Math.hypot(r.goal.x - r.x, r.goal.y - r.y);
      goalHtml =
        `<div class="robot-goal">goal: (${r.goal.x.toFixed(1)}, ${r.goal.y.toFixed(1)})</div>` +
        `<div class="robot-dist">distance: ${d.toFixed(2)} m</div>`;
    }
    return `<div class="robot-card">
      <div class="robot-name" style="color:${colors[i % colors.length]}">${r.id}</div>
      <div class="robot-pos">x: ${r.x.toFixed(2)}  y: ${r.y.toFixed(2)}  θ: ${(r.theta * 180 / Math.PI).toFixed(0)}°</div>
      ${goalHtml}
      <span class="robot-status ${statusClass}">${r.status}</span>
      <div class="battery-bar"><div class="battery-fill" style="width:${r.battery}%;background:${batColor}"></div></div>
    </div>`;
  }).join('');
}

// --- Interaction: click, drag, zoom ---
let dragging = false, dragStart = null, dragDidMove = false;

canvas.addEventListener('mousedown', (e) => {
  dragging = true;
  dragDidMove = false;
  dragStart = { x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy };
});
canvas.addEventListener('mousemove', (e) => {
  if (!dragging) return;
  const dx = e.clientX - dragStart.x;
  const dy = e.clientY - dragStart.y;
  if (Math.abs(dx) + Math.abs(dy) > 3) dragDidMove = true;
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const spanY = view.span * (canvas.height / canvas.width);
  view.cx = dragStart.cx - (dx * scaleX / canvas.width) * view.span;
  view.cy = dragStart.cy + (dy * scaleX / canvas.height) * spanY;
  view.follow = false;
  render();
});
canvas.addEventListener('mouseup', (e) => {
  dragging = false;
  if (dragDidMove) return;
  // Treat as click: send nearest robot to that point
  const rect = canvas.getBoundingClientRect();
  const px = (e.clientX - rect.left) * (canvas.width / rect.width);
  const py = (e.clientY - rect.top) * (canvas.height / rect.height);
  const [wx, wy] = canvasToWorld(px, py);
  let nearest = null, minDist = Infinity;
  robots.forEach(r => {
    if (!r.online) return;
    const d = Math.hypot(r.x - wx, r.y - wy);
    if (d < minDist) { minDist = d; nearest = r; }
  });
  if (nearest && ws && ws.readyState === 1) {
    ws.send(JSON.stringify({ command: 'navigate', robot_id: nearest.id, x: wx, y: wy }));
    addChatMessage(`Sent ${nearest.id} to (${wx.toFixed(1)}, ${wy.toFixed(1)})`, 'tool');
  }
});
canvas.addEventListener('mouseleave', () => { dragging = false; });

canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const px = (e.clientX - rect.left) * (canvas.width / rect.width);
  const py = (e.clientY - rect.top) * (canvas.height / rect.height);
  const [wxBefore, wyBefore] = canvasToWorld(px, py);
  const factor = e.deltaY < 0 ? 0.85 : 1.18;
  view.span = Math.max(20, Math.min(1600, view.span * factor));
  const [wxAfter, wyAfter] = canvasToWorld(px, py);
  view.cx += wxBefore - wxAfter;
  view.cy += wyBefore - wyAfter;
  render();
}, { passive: false });

document.getElementById('btn-zoom-in').onclick = () => { view.span = Math.max(20, view.span * 0.7); render(); };
document.getElementById('btn-zoom-out').onclick = () => { view.span = Math.min(1600, view.span * 1.4); render(); };
document.getElementById('btn-fit-map').onclick = () => { view.cx = 0; view.cy = 0; view.span = 1250; view.follow = false; render(); };
document.getElementById('btn-follow').onclick = () => {
  view.follow = !view.follow;
  if (view.follow && robots.length) { view.cx = robots[0].x; view.cy = robots[0].y; view.span = Math.min(view.span, 80); }
  render();
};

// --- Chat ---
const chatMessages = document.getElementById('chat-messages');
const chatInput = document.getElementById('chat-input');
const chatSend = document.getElementById('chat-send');

function addChatMessage(text, type) {
  const div = document.createElement('div');
  div.className = `msg msg-${type}`;
  if (type === 'bot' && typeof marked !== 'undefined') {
    div.innerHTML = marked.parse(text);
  } else {
    div.textContent = text;
  }
  chatMessages.appendChild(div);
  chatMessages.scrollTop = chatMessages.scrollHeight;
}
function sendChat() {
  const text = chatInput.value.trim();
  if (!text) return;
  chatInput.value = '';
  addChatMessage(text, 'user');
  if (ws && ws.readyState === 1) {
    ws.send(JSON.stringify({ command: 'chat', message: text }));
    const typing = document.createElement('div');
    typing.className = 'typing';
    typing.id = 'typing-indicator';
    typing.textContent = 'Thinking...';
    chatMessages.appendChild(typing);
    chatMessages.scrollTop = chatMessages.scrollHeight;
  } else {
    addChatMessage('Not connected to server', 'tool');
  }
}
chatSend.addEventListener('click', sendChat);
chatInput.addEventListener('keydown', (e) => { if (e.key === 'Enter') sendChat(); });

window.addEventListener('resize', render);

// Refresh the Gazebo chase-cam preview image (cache-busted)
const refreshCamBtn = document.getElementById('btn-refresh-cam');
const chaseImg = document.getElementById('pguard-chase-img');
function refreshChase() {
  if (chaseImg) chaseImg.src = 'pguard_chase.png?t=' + Date.now();
}
if (refreshCamBtn && chaseImg) {
  refreshCamBtn.addEventListener('click', refreshChase);
  // Auto-refresh every 3.5s so it stays live
  setInterval(refreshChase, 3500);
}

connect();
render();
