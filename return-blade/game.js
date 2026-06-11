'use strict';

/* ========== 回刃 RETURN BLADE ==========
 * 唯一操作：点击 → 飞刃掷出并原路返回，沿途斩敌。
 * 唯一矛盾：飞刃离手期间核心毫无防御。
 * 计分：单次飞行第 n 杀得 n 分，贪心即风险。
 */

const CFG = {
  coreR: 18,
  maxHp: 3,
  bladeR: 15,
  outSpd: 1050,        // 出刃速度 px/s
  backSpd: 800,        // 回收速度 px/s
  minThrow: 70,        // 最短投掷距离
  spawn0: 2.1,         // 初始刷怪间隔（秒）
  spawnMin: 0.55,      // 间隔下限
  spawnRamp: 0.014,    // 间隔随时间缩短
  baseSpd: 55,         // 敌人基础速度
  spdRamp: 1.3,        // 敌速随时间增长
  spdCap: 165,
  graceTime: 1.2,      // 开局喘息
};

const COLORS = {
  bg: '#0a0d14',
  core: '#6ef3ff',
  blade: '#e8f6ff',
  drone: '#ff5470',
  swift: '#ffd644',
  heavy: '#b07cff',
  text: '#cfd8e3',
  dim: '#5a6577',
  danger: '#ff3b3b',
};

// ---------- canvas ----------
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
let W = 0, H = 0;
function resize() {
  const dpr = window.devicePixelRatio || 1;
  W = window.innerWidth; H = window.innerHeight;
  canvas.width = W * dpr; canvas.height = H * dpr;
  canvas.style.width = W + 'px'; canvas.style.height = H + 'px';
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener('resize', resize);
resize();
const CX = () => W / 2, CY = () => H / 2;

// ---------- audio（WebAudio 合成，无素材） ----------
let AC = null, muted = false;
function ensureAudio() {
  if (!AC) {
    try { AC = new (window.AudioContext || window.webkitAudioContext)(); } catch (e) { AC = null; }
  }
  if (AC && AC.state === 'suspended') AC.resume();
}
function beep(freq, dur, type, vol, slide) {
  if (!AC || muted) return;
  const t = AC.currentTime;
  const o = AC.createOscillator(), g = AC.createGain();
  o.type = type || 'square';
  o.frequency.setValueAtTime(freq, t);
  if (slide) o.frequency.exponentialRampToValueAtTime(Math.max(30, freq + slide), t + dur);
  g.gain.setValueAtTime(vol || 0.07, t);
  g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
  o.connect(g); g.connect(AC.destination);
  o.start(t); o.stop(t + dur);
}

// ---------- 状态 ----------
let state = 'menu';            // menu | play | over | pause
let best = +(localStorage.getItem('rb_best') || 0);
let bestCombo = +(localStorage.getItem('rb_bestcombo') || 0);
let score, hp, t, enemies, parts, texts, blade, combo, runMaxCombo;
let shake, freeze, spawnAcc, invuln, overT, newBest;

function reset() {
  score = 0; hp = CFG.maxHp; t = 0;
  enemies = []; parts = []; texts = [];
  combo = 0; runMaxCombo = 0;
  shake = 0; freeze = 0; spawnAcc = -CFG.graceTime; invuln = 0;
  overT = 0; newBest = false;
  blade = { state: 'idle', x: CX(), y: CY(), tx: 0, ty: 0, spin: 0, trail: [] };
}
reset();

// ---------- 输入 ----------
canvas.addEventListener('pointerdown', (e) => {
  e.preventDefault();
  ensureAudio();
  const x = e.clientX, y = e.clientY;
  if (state === 'menu') { reset(); state = 'play'; beep(440, 0.12, 'triangle', 0.08, 220); return; }
  if (state === 'over') {
    if (overT > 0.6) { reset(); state = 'play'; beep(440, 0.12, 'triangle', 0.08, 220); }
    return;
  }
  if (state === 'pause') { state = 'play'; return; }
  if (state === 'play') throwBlade(x, y);
});
window.addEventListener('keydown', (e) => {
  if (e.key === 'm' || e.key === 'M') muted = !muted;
  if ((e.key === 'p' || e.key === 'P') && (state === 'play' || state === 'pause')) {
    state = state === 'play' ? 'pause' : 'play';
  }
  if ((e.key === 'r' || e.key === 'R') && state === 'over') { reset(); state = 'play'; }
});
document.addEventListener('visibilitychange', () => {
  if (document.hidden && state === 'play') state = 'pause';
});

// ---------- 飞刃 ----------
function throwBlade(px, py) {
  if (blade.state !== 'idle') return;
  let dx = px - CX(), dy = py - CY();
  let d = Math.hypot(dx, dy);
  if (d < 1) { dx = 1; dy = 0; d = 1; }
  if (d < CFG.minThrow) { dx *= CFG.minThrow / d; dy *= CFG.minThrow / d; }
  blade.tx = CX() + dx; blade.ty = CY() + dy;
  blade.x = CX(); blade.y = CY();
  blade.state = 'out';
  blade.trail.length = 0;
  combo = 0;
  beep(200, 0.1, 'sawtooth', 0.045, 360);
}

function updateBlade(dt) {
  const b = blade;
  if (b.state === 'idle') { b.x = CX(); b.y = CY(); b.spin += dt * 3; return; }
  b.spin += dt * 22;
  const tx = b.state === 'out' ? b.tx : CX();
  const ty = b.state === 'out' ? b.ty : CY();
  const spd = b.state === 'out' ? CFG.outSpd : CFG.backSpd;
  const dx = tx - b.x, dy = ty - b.y;
  const d = Math.hypot(dx, dy), step = spd * dt;
  if (d <= step) {
    b.x = tx; b.y = ty;
    if (b.state === 'out') {
      b.state = 'back';
    } else {
      b.state = 'idle';
      if (combo >= 4) {
        addText(CX(), CY() - 46, '完美回收 ×' + combo, COLORS.core, 22);
        beep(660, 0.16, 'triangle', 0.08, 440);
      }
      combo = 0;
    }
  } else {
    b.x += dx / d * step; b.y += dy / d * step;
  }
  b.trail.push({ x: b.x, y: b.y, a: 1 });
  if (b.trail.length > 18) b.trail.shift();
}

// ---------- 敌人 ----------
function spawnCluster() {
  const ang = Math.random() * Math.PI * 2;
  const R = Math.hypot(W, H) / 2 + 50;
  const n = 1 + Math.floor(Math.random() * Math.min(5, 1 + t / 20));
  for (let i = 0; i < n; i++) {
    const a = ang + (Math.random() - 0.5) * 0.55;
    const x = CX() + Math.cos(a) * (R + Math.random() * 70);
    const y = CY() + Math.sin(a) * (R + Math.random() * 70);
    const roll = Math.random();
    let type = 'drone', r = 13, mult = 1, ehp = 1;
    if (t > 45 && roll < 0.12) { type = 'heavy'; r = 21; mult = 0.5; ehp = 2; }
    else if (roll < Math.min(0.35, 0.04 + t * 0.004)) { type = 'swift'; r = 9; mult = 1.75; }
    const spd = Math.min(CFG.spdCap, CFG.baseSpd + t * CFG.spdRamp) * mult * (0.9 + Math.random() * 0.2);
    enemies.push({ x, y, r, spd, hp: ehp, type, hitT: 0, wob: Math.random() * 6.28 });
  }
}

function updateEnemies(dt) {
  for (let i = enemies.length - 1; i >= 0; i--) {
    const e = enemies[i];
    e.hitT = Math.max(0, e.hitT - dt);
    e.wob += dt * 4;
    const dx = CX() - e.x, dy = CY() - e.y;
    const d = Math.hypot(dx, dy) || 1;
    e.x += dx / d * e.spd * dt;
    e.y += dy / d * e.spd * dt;

    // 飞刃命中（去程 + 回程）
    if (blade.state !== 'idle' && e.hitT <= 0 &&
        Math.hypot(blade.x - e.x, blade.y - e.y) < CFG.bladeR + e.r) {
      e.hp--;
      e.hitT = 0.35;
      if (e.hp <= 0) {
        killEnemy(e, i);
        continue;
      } else {
        burst(e.x, e.y, '#9aa6b8', 6);
        addText(e.x, e.y - e.r - 6, '叮', '#9aa6b8', 14);
        beep(140, 0.07, 'square', 0.06);
        shake = Math.max(shake, 3);
      }
    }

    // 触核
    if (Math.hypot(dx, dy) < CFG.coreR + e.r) {
      enemies.splice(i, 1);
      burst(e.x, e.y, COLORS.danger, 14);
      if (invuln <= 0) {
        hp--;
        invuln = 1.0;
        shake = 16;
        freeze = 0.06;
        beep(80, 0.3, 'sawtooth', 0.12, -40);
        if (hp <= 0) gameOver();
      }
    }
  }
}

function killEnemy(e, idx) {
  enemies.splice(idx, 1);
  combo++;
  score += combo;
  if (combo > runMaxCombo) runMaxCombo = combo;
  const col = COLORS[e.type];
  burst(e.x, e.y, col, 12);
  addText(e.x, e.y - e.r - 8, '+' + combo, col, 15 + Math.min(14, combo * 2));
  freeze = Math.min(0.09, 0.03 + combo * 0.005);   // 打击停顿，连斩越高越爽
  shake = Math.max(shake, 2 + combo);
  beep(280 * Math.pow(1.1, Math.min(combo, 16)), 0.09, 'square', 0.07, 120);
}

function gameOver() {
  state = 'over';
  overT = 0;
  newBest = score > best;
  if (newBest) { best = score; localStorage.setItem('rb_best', best); }
  if (runMaxCombo > bestCombo) { bestCombo = runMaxCombo; localStorage.setItem('rb_bestcombo', bestCombo); }
  beep(60, 0.6, 'sawtooth', 0.12, -20);
}

// ---------- 粒子 / 浮动文字 ----------
function burst(x, y, col, n) {
  for (let i = 0; i < n; i++) {
    const a = Math.random() * Math.PI * 2, s = 60 + Math.random() * 220;
    parts.push({ x, y, vx: Math.cos(a) * s, vy: Math.sin(a) * s, life: 0.3 + Math.random() * 0.35, t: 0, col, r: 1.5 + Math.random() * 2.5 });
  }
}
function addText(x, y, str, col, size) {
  texts.push({ x, y, str, col, size, t: 0, life: 0.9 });
}
function updateFx(dt) {
  for (let i = parts.length - 1; i >= 0; i--) {
    const p = parts[i];
    p.t += dt;
    if (p.t >= p.life) { parts.splice(i, 1); continue; }
    p.x += p.vx * dt; p.y += p.vy * dt;
    p.vx *= Math.pow(0.02, dt); p.vy *= Math.pow(0.02, dt);
  }
  for (let i = texts.length - 1; i >= 0; i--) {
    const x = texts[i];
    x.t += dt;
    x.y -= 26 * dt;
    if (x.t >= x.life) texts.splice(i, 1);
  }
}

// ---------- 主循环 ----------
let last = performance.now();
function loop(now) {
  let dt = Math.min(0.05, (now - last) / 1000);
  last = now;

  if (state === 'play') {
    if (freeze > 0) { freeze -= dt; dt = 0; }   // 打击停顿
    t += dt;
    invuln = Math.max(0, invuln - dt);
    spawnAcc += dt;
    const interval = Math.max(CFG.spawnMin, CFG.spawn0 - t * CFG.spawnRamp);
    while (spawnAcc >= interval) { spawnAcc -= interval; spawnCluster(); }
    updateBlade(dt);
    updateEnemies(dt);
    updateFx(dt);
  } else if (state === 'over') {
    overT += dt;
    updateFx(dt);
  }
  shake = Math.max(0, shake - 60 * dt * (0.3 + shake * 0.04));

  render();
  requestAnimationFrame(loop);
}

// ---------- 渲染 ----------
function render() {
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = COLORS.bg;
  ctx.fillRect(0, 0, W, H);

  // 暗角
  const vg = ctx.createRadialGradient(CX(), CY(), Math.min(W, H) * 0.25, CX(), CY(), Math.max(W, H) * 0.75);
  vg.addColorStop(0, 'rgba(0,0,0,0)');
  vg.addColorStop(1, 'rgba(0,0,0,0.55)');
  ctx.fillStyle = vg;
  ctx.fillRect(0, 0, W, H);

  ctx.save();
  if (shake > 0.5) ctx.translate((Math.random() - 0.5) * shake, (Math.random() - 0.5) * shake);

  if (state !== 'menu') {
    drawCore();
    drawEnemies();
    drawBlade();
    drawFx();
  }
  ctx.restore();

  if (state === 'menu') drawMenu();
  else drawHud();
  if (state === 'over') drawOver();
  if (state === 'pause') drawPause();
}

function drawCore() {
  const blink = invuln > 0 && Math.floor(invuln * 12) % 2 === 0;

  // 飞刃离手 → 危险脉冲圈
  if (blade.state !== 'idle' && state === 'play') {
    const pulse = 0.5 + 0.5 * Math.sin(t * 10);
    ctx.strokeStyle = 'rgba(255,59,59,' + (0.25 + 0.3 * pulse) + ')';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(CX(), CY(), CFG.coreR + 10 + pulse * 5, 0, Math.PI * 2);
    ctx.stroke();
  }

  ctx.save();
  ctx.shadowColor = COLORS.core;
  ctx.shadowBlur = 22;
  ctx.fillStyle = blink ? '#ffffff' : COLORS.core;
  ctx.beginPath();
  ctx.arc(CX(), CY(), CFG.coreR, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = '#0a0d14';
  ctx.beginPath();
  ctx.arc(CX(), CY(), CFG.coreR * 0.55, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawEnemies() {
  for (const e of enemies) {
    const col = COLORS[e.type];
    const wob = Math.sin(e.wob) * 1.5;
    ctx.save();
    ctx.shadowColor = col;
    ctx.shadowBlur = 12;
    ctx.fillStyle = e.hitT > 0.25 ? '#ffffff' : col;
    ctx.beginPath();
    ctx.arc(e.x + wob, e.y, e.r, 0, Math.PI * 2);
    ctx.fill();
    if (e.type === 'heavy' && e.hp === 1) {  // 重甲破裂
      ctx.strokeStyle = '#0a0d14';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(e.x + wob - e.r * 0.5, e.y - e.r * 0.5);
      ctx.lineTo(e.x + wob + e.r * 0.3, e.y + e.r * 0.6);
      ctx.stroke();
    }
    ctx.restore();
  }
}

function drawBlade() {
  const b = blade;
  // 轨迹
  if (b.state !== 'idle' && b.trail.length > 1) {
    ctx.save();
    ctx.strokeStyle = 'rgba(232,246,255,0.35)';
    ctx.lineCap = 'round';
    for (let i = 1; i < b.trail.length; i++) {
      ctx.lineWidth = (i / b.trail.length) * 6;
      ctx.globalAlpha = i / b.trail.length * 0.6;
      ctx.beginPath();
      ctx.moveTo(b.trail[i - 1].x, b.trail[i - 1].y);
      ctx.lineTo(b.trail[i].x, b.trail[i].y);
      ctx.stroke();
    }
    ctx.restore();
  }
  // 旋转三叶刃
  ctx.save();
  ctx.translate(b.x, b.y);
  ctx.rotate(b.spin);
  ctx.shadowColor = COLORS.blade;
  ctx.shadowBlur = 14;
  ctx.fillStyle = COLORS.blade;
  const R = b.state === 'idle' ? CFG.bladeR * 0.7 : CFG.bladeR;
  for (let k = 0; k < 3; k++) {
    ctx.rotate(Math.PI * 2 / 3);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.quadraticCurveTo(R * 0.5, -R * 0.45, R, 0);
    ctx.quadraticCurveTo(R * 0.5, R * 0.2, 0, 0);
    ctx.fill();
  }
  ctx.beginPath();
  ctx.arc(0, 0, R * 0.22, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawFx() {
  for (const p of parts) {
    ctx.globalAlpha = 1 - p.t / p.life;
    ctx.fillStyle = p.col;
    ctx.beginPath();
    ctx.arc(p.x, p.y, p.r, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.globalAlpha = 1;
  for (const x of texts) {
    ctx.globalAlpha = 1 - x.t / x.life;
    ctx.fillStyle = x.col;
    ctx.font = 'bold ' + x.size + 'px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(x.str, x.x, x.y);
  }
  ctx.globalAlpha = 1;
}

function drawHud() {
  ctx.textAlign = 'left';
  ctx.fillStyle = COLORS.text;
  ctx.font = 'bold 26px sans-serif';
  ctx.fillText('' + score, 20, 40);
  ctx.fillStyle = COLORS.dim;
  ctx.font = '13px sans-serif';
  ctx.fillText('最佳 ' + best + ' · 最大连斩 ×' + bestCombo, 20, 60);

  // 核心完整度
  for (let i = 0; i < CFG.maxHp; i++) {
    ctx.save();
    ctx.translate(W - 26 - i * 26, 34);
    ctx.rotate(Math.PI / 4);
    ctx.fillStyle = i < hp ? COLORS.core : '#2a3242';
    ctx.fillRect(-7, -7, 14, 14);
    ctx.restore();
  }

  // 飞行中连斩计数
  if (state === 'play' && blade.state !== 'idle' && combo > 0) {
    ctx.textAlign = 'center';
    ctx.fillStyle = COLORS.swift;
    ctx.font = 'bold ' + (20 + Math.min(16, combo * 2)) + 'px sans-serif';
    ctx.fillText('连斩 ×' + combo, CX(), 48);
  }

  ctx.textAlign = 'right';
  ctx.fillStyle = COLORS.dim;
  ctx.font = '12px sans-serif';
  ctx.fillText('M 静音 · P 暂停', W - 16, H - 14);
}

function drawMenu() {
  ctx.textAlign = 'center';
  ctx.save();
  ctx.shadowColor = COLORS.core;
  ctx.shadowBlur = 30;
  ctx.fillStyle = COLORS.core;
  ctx.font = 'bold 64px sans-serif';
  ctx.fillText('回 刃', CX(), CY() - 110);
  ctx.restore();
  ctx.fillStyle = COLORS.dim;
  ctx.font = '16px sans-serif';
  ctx.fillText('R E T U R N   B L A D E', CX(), CY() - 76);

  ctx.fillStyle = COLORS.text;
  ctx.font = '16px sans-serif';
  const lines = [
    '点击任意位置投出飞刃，飞刃沿原路返回',
    '去程与回程都会斩杀沿途敌人',
    '单次飞行第 n 杀得 n 分 —— 连斩越长，得分越爆炸',
    '但飞刃离手时，核心毫无防御',
  ];
  lines.forEach((s, i) => ctx.fillText(s, CX(), CY() - 20 + i * 28));

  ctx.fillStyle = COLORS.swift;
  ctx.font = 'bold 20px sans-serif';
  ctx.fillText('贪心，还是稳妥？', CX(), CY() + 110);

  const blink = Math.floor(performance.now() / 600) % 2 === 0;
  if (blink) {
    ctx.fillStyle = COLORS.core;
    ctx.font = '18px sans-serif';
    ctx.fillText('—— 点击开始 ——', CX(), CY() + 160);
  }
  if (best > 0) {
    ctx.fillStyle = COLORS.dim;
    ctx.font = '13px sans-serif';
    ctx.fillText('历史最佳 ' + best + ' · 最大连斩 ×' + bestCombo, CX(), CY() + 200);
  }
}

function drawOver() {
  ctx.fillStyle = 'rgba(10,13,20,0.72)';
  ctx.fillRect(0, 0, W, H);
  ctx.textAlign = 'center';
  ctx.save();
  ctx.shadowColor = COLORS.danger;
  ctx.shadowBlur = 24;
  ctx.fillStyle = COLORS.danger;
  ctx.font = 'bold 48px sans-serif';
  ctx.fillText('核 心 破 碎', CX(), CY() - 60);
  ctx.restore();

  ctx.fillStyle = COLORS.text;
  ctx.font = 'bold 28px sans-serif';
  ctx.fillText('本局分数  ' + score, CX(), CY() + 4);
  if (newBest) {
    ctx.fillStyle = COLORS.swift;
    ctx.font = 'bold 18px sans-serif';
    ctx.fillText('★ 新纪录！', CX(), CY() + 34);
  }
  ctx.fillStyle = COLORS.dim;
  ctx.font = '15px sans-serif';
  ctx.fillText('本局最大连斩 ×' + runMaxCombo + ' · 历史最佳 ' + best, CX(), CY() + 64);

  if (overT > 0.6 && Math.floor(performance.now() / 600) % 2 === 0) {
    ctx.fillStyle = COLORS.core;
    ctx.font = '17px sans-serif';
    ctx.fillText('点击再来一局', CX(), CY() + 120);
  }
}

function drawPause() {
  ctx.fillStyle = 'rgba(10,13,20,0.6)';
  ctx.fillRect(0, 0, W, H);
  ctx.textAlign = 'center';
  ctx.fillStyle = COLORS.text;
  ctx.font = 'bold 32px sans-serif';
  ctx.fillText('已暂停', CX(), CY());
  ctx.fillStyle = COLORS.dim;
  ctx.font = '15px sans-serif';
  ctx.fillText('点击或按 P 继续', CX(), CY() + 36);
}

requestAnimationFrame(loop);
