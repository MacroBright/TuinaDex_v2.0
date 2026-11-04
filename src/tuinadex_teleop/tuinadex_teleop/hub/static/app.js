// ============================================================
// TuinaDex v2.0 Embodied AI Data Collection Workstation App
// ============================================================

// 1. DOM 核心元素索引
const stateEl = document.getElementById('state');
const statsEl = document.getElementById('stats');
const episodeEl = document.getElementById('episode');
const noticeEl = document.getElementById('notice');
const quotaCountEl = document.getElementById('quota-count');
const sessionSummaryEl = document.getElementById('session-summary');
const episodeListEl = document.getElementById('episode-list');

// OSD 悬浮层
const osdInstructionEl = document.getElementById('osd-instruction');
const osdPhaseEl = document.getElementById('osd-phase');
const osdTimerEl = document.getElementById('osd-timer');
const osdFpsEl = document.getElementById('osd-fps');
const osdTotalForceEl = document.getElementById('osd-total-force');
const primaryViewportEl = document.getElementById('primary-viewport');
const primaryImgEl = document.getElementById('primary-img');
const primaryViewNameEl = document.getElementById('primary-view-name');
const secondaryStripEl = document.getElementById('secondary-strip');

// 表单输入
const instructionInput = document.getElementById('instruction');
const phaseInput = document.getElementById('phase');

// 2. Web Audio 音频反馈系统 (ALOHA 风格音效)
class AudioFeedback {
  constructor() {
    this.ctx = null;
  }

  _init() {
    if (!this.ctx) {
      const AudioCtx = window.AudioContext || window.webkitAudioContext;
      if (AudioCtx) this.ctx = new AudioCtx();
    }
    if (this.ctx && this.ctx.state === 'suspended') {
      this.ctx.resume();
    }
  }

  playTone(freq, type = 'sine', duration = 0.1, endFreq = null) {
    try {
      this._init();
      if (!this.ctx) return;
      const osc = this.ctx.createOscillator();
      const gain = this.ctx.createGain();
      osc.type = type;
      osc.frequency.setValueAtTime(freq, this.ctx.currentTime);
      if (endFreq) {
        osc.frequency.exponentialRampToValueAtTime(endFreq, this.ctx.currentTime + duration);
      }
      gain.gain.setValueAtTime(0.15, this.ctx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, this.ctx.currentTime + duration);
      osc.connect(gain);
      gain.connect(this.ctx.destination);
      osc.start();
      osc.stop(this.ctx.currentTime + duration);
    } catch (_) {}
  }

  recordStart() {
    // 升调双音 (880Hz -> 1175Hz)
    this.playTone(880, 'sine', 0.1, 1175);
  }

  recordStop() {
    // 降调双音 (880Hz -> 587Hz)
    this.playTone(880, 'sine', 0.1, 587);
  }

  saveSuccess() {
    // 愉悦三和弦 C5-E5-G5
    setTimeout(() => this.playTone(523.25, 'triangle', 0.08), 0);
    setTimeout(() => this.playTone(659.25, 'triangle', 0.08), 80);
    setTimeout(() => this.playTone(783.99, 'triangle', 0.15), 160);
  }

  discard() {
    // 低沉提示音
    this.playTone(180, 'sawtooth', 0.15);
  }

  phaseBlip() {
    // 短促阶段切换音
    this.playTone(987.77, 'sine', 0.04);
  }

  forceAlarm() {
    // 超限急促嗡鸣
    this.playTone(220, 'square', 0.12);
  }
}

const audio = new AudioFeedback();

// 3. 多相机主次机位管理 (Focus + Thumbnails)
const CAMERA_METAS = {
  overhead: { name: 'OVERHEAD 全局俯视', fov: '60°', tag: '全局空间全景' },
  wrist: { name: 'WRIST 腕部第一视角', fov: '75°', tag: '随动手部特写' },
  side: { name: 'SIDE 侧向水平视角', fov: '45°', tag: '下压深度切面' },
};

let primaryCam = 'wrist';
const allCams = ['wrist', 'overhead', 'side'];

function renderCameraLayout() {
  const meta = CAMERA_METAS[primaryCam] || { name: primaryCam.toUpperCase(), fov: '', tag: '' };
  primaryImgEl.src = `/video/${primaryCam}`;
  primaryViewNameEl.textContent = `${meta.name} · ${meta.fov}`;

  // 渲染副机位缩略图
  secondaryStripEl.innerHTML = '';
  allCams.forEach((cam) => {
    if (cam === primaryCam) return;
    const cMeta = CAMERA_METAS[cam] || { name: cam.toUpperCase(), fov: '', tag: '' };
    const fig = document.createElement('figure');
    fig.className = 'secondary-thumb';
    fig.dataset.cam = cam;
    fig.title = `点击设为主视角 (Tab)`;
    fig.innerHTML = `
      <img src="/video/${cam}" alt="${cMeta.name}"/>
      <figcaption>
        <span>${cam.toUpperCase()}</span>
        <strong>${cMeta.tag} · ${cMeta.fov}</strong>
      </figcaption>
      <div class="thumb-swap-overlay">设为主视角</div>
    `;
    fig.addEventListener('click', () => swapPrimaryCam(cam));
    secondaryStripEl.appendChild(fig);
  });
}

function swapPrimaryCam(targetCam) {
  if (!targetCam || targetCam === primaryCam) {
    // 默认切到下一个
    const others = allCams.filter((c) => c !== primaryCam);
    targetCam = others[0];
  }
  primaryCam = targetCam;
  renderCameraLayout();
  audio.phaseBlip();
}

renderCameraLayout();

// 4. 22 维生理学解剖分组卡片构建
const armJointsEl = document.getElementById('arm-joints');
const handThumbEl = document.getElementById('hand-thumb');
const handIndexEl = document.getElementById('hand-index');
const handMiddleEl = document.getElementById('hand-middle');
const handRingEl = document.getElementById('hand-ring');

const ARM_NAMES = ['J0 基座', 'J1 大臂', 'J2 肘部', 'J3 俯仰', 'J4 偏航', 'J5 翻转'];
const FINGER_JOINT_NAMES = ['MCP-S', 'MCP-F', 'PIP', 'DIP'];

function makeBarRow(label) {
  const row = document.createElement('div');
  row.className = 'bar-row';
  row.setAttribute('role', 'listitem');

  const name = document.createElement('span');
  name.className = 'bar-label';
  name.textContent = label;

  const track = document.createElement('span');
  track.className = 'bar-track';
  const fill = document.createElement('span');
  fill.className = 'bar-fill';
  track.appendChild(fill);

  const value = document.createElement('span');
  value.className = 'bar-value';
  value.textContent = '0.00';

  row.append(name, track, value);
  return row;
}

// 填充 DOM
ARM_NAMES.forEach((n) => armJointsEl.appendChild(makeBarRow(n)));
FINGER_JOINT_NAMES.forEach((n) => handThumbEl.appendChild(makeBarRow(n)));
FINGER_JOINT_NAMES.forEach((n) => handIndexEl.appendChild(makeBarRow(n)));
FINGER_JOINT_NAMES.forEach((n) => handMiddleEl.appendChild(makeBarRow(n)));
FINGER_JOINT_NAMES.forEach((n) => handRingEl.appendChild(makeBarRow(n)));

function updateGroupBars(container, values, lower, upper) {
  const rows = container.children;
  for (let i = 0; i < rows.length && i < values.length; i += 1) {
    const value = Number(values[i]);
    if (!Number.isFinite(value)) continue;
    const ratio = Math.max(0, Math.min(1, (value - lower) / (upper - lower)));
    const fill = rows[i].querySelector('.bar-fill');
    fill.style.width = `${(ratio * 100).toFixed(1)}%`;
    if (Math.abs(value) > 2.8) {
      fill.style.background = 'var(--bad)';
    } else {
      fill.style.background = 'var(--accent)';
    }
    rows[i].querySelector('.bar-value').textContent = value.toFixed(2);
  }
}

// 5. 指尖受力气泡雷达
const FINGER_KEYS = ['index', 'middle', 'ring', 'thumb'];
const forceSummaryBadge = document.getElementById('force-summary-badge');

function updateFingerRadar(forces) {
  let total = 0;
  FINGER_KEYS.forEach((fKey) => {
    const fVal = Math.max(0, Number(forces[fKey] || 0));
    total += fVal;
    const bubble = document.querySelector(`.finger-bubble[data-finger="${fKey}"] .bubble-circle`);
    const valText = document.querySelector(`.finger-bubble[data-finger="${fKey}"] .force-val`);
    if (bubble && valText) {
      valText.textContent = fVal.toFixed(1);
      if (fVal > 35) {
        bubble.style.background = 'rgba(239, 68, 68, 0.4)';
        bubble.style.borderColor = 'var(--bad)';
        bubble.style.transform = 'scale(1.15)';
      } else if (fVal > 15) {
        bubble.style.background = 'rgba(245, 158, 11, 0.3)';
        bubble.style.borderColor = 'var(--warn)';
        bubble.style.transform = 'scale(1.08)';
      } else if (fVal > 1.0) {
        bubble.style.background = 'rgba(16, 185, 129, 0.25)';
        bubble.style.borderColor = 'var(--accent)';
        bubble.style.transform = 'scale(1.0)';
      } else {
        bubble.style.background = 'rgba(16, 185, 129, 0.08)';
        bubble.style.borderColor = 'var(--border)';
        bubble.style.transform = 'scale(1.0)';
      }
    }
  });

  osdTotalForceEl.textContent = total.toFixed(1);
  forceSummaryBadge.textContent = `${total.toFixed(1)} N`;

  // 超限警报音
  if (total > 45) {
    audio.forceAlarm();
  }
}

// 6. 手法阶段切换逻辑 (按键 1-4)
const PHASE_MAP = {
  1: { id: 'approach', label: '1. 准备接近 (Approach)' },
  2: { id: 'press', label: '2. 定位下压 (Press)' },
  3: { id: 'knead', label: '3. 循环推揉 (Knead)' },
  4: { id: 'retract', label: '4. 阻尼抬手 (Retract)' },
};

function setPhaseByNum(num) {
  const p = PHASE_MAP[num];
  if (!p) return;
  document.querySelectorAll('.phase-btn').forEach((btn) => {
    btn.classList.toggle('active', btn.dataset.num === String(num));
  });
  phaseInput.value = p.id;
  osdPhaseEl.textContent = `阶段: [${p.label}]`;
  audio.phaseBlip();
  act('label');
}

document.querySelectorAll('.phase-btn').forEach((btn) => {
  btn.addEventListener('click', () => setPhaseByNum(btn.dataset.num));
});

// 7. 采集状态机与试次历史管理
let lastState = 'IDLE';
let lastSavedCount = 0;
let lastDiscardedCount = 0;
let recStartTime = 0;
let timerInterval = null;

function setControlAvailability(state) {
  const allowed = {
    IDLE: new Set(['start', 'home']),
    RECORDING: new Set(['pause', 'stop']),
    PAUSED: new Set(['resume', 'stop', 'home']),
    REVIEWING: new Set(['save', 'discard', 'home']),
    SAVING: new Set(),
  };
  document.querySelectorAll('[data-act]').forEach((button) => {
    if (button.dataset.act === 'label') return;
    button.disabled = !(allowed[state] || new Set()).has(button.dataset.act);
  });
}

function addEpisodeRecord(epId, statusTag, isSaved) {
  const empty = episodeListEl.querySelector('.empty-hint');
  if (empty) empty.remove();

  const item = document.createElement('div');
  item.className = 'episode-item';
  item.innerHTML = `
    <span class="episode-id">#${String(epId).padStart(3, '0')}</span>
    <span class="episode-meta">${instructionInput.value} · ${phaseInput.value}</span>
    <span class="${isSaved ? 'episode-tag-saved' : 'episode-tag-discarded'}">${statusTag}</span>
  `;
  episodeListEl.prepend(item);
}

function renderStatus(status) {
  const curState = status.state;
  stateEl.textContent = curState;
  stateEl.dataset.state = curState;
  episodeEl.textContent = `EP ${String(status.episode_index || 0).padStart(2, '0')}`;
  quotaCountEl.textContent = status.saved || 0;
  sessionSummaryEl.textContent = `已存 ${status.saved || 0} · 丢弃 ${status.discarded || 0}`;

  // 状态迁移检测与音效触发
  if (curState !== lastState) {
    if (curState === 'RECORDING') {
      audio.recordStart();
      recStartTime = Date.now();
      primaryViewportEl.classList.add('recording-active');
      if (!timerInterval) {
        timerInterval = setInterval(() => {
          const elapsed = (Date.now() - recStartTime) / 1000;
          const mins = String(Math.floor(elapsed / 60)).padStart(2, '0');
          const secs = (elapsed % 60).toFixed(1).padStart(4, '0');
          osdTimerEl.textContent = `${mins}:${secs}`;
        }, 100);
      }
    } else {
      primaryViewportEl.classList.remove('recording-active');
      if (timerInterval) {
        clearInterval(timerInterval);
        timerInterval = null;
      }
      if (lastState === 'RECORDING') {
        audio.recordStop();
      }
    }

    if (curState === 'SAVING') {
      audio.saveSuccess();
    }
    lastState = curState;
  }

  // 试次结算追踪
  if (status.saved > lastSavedCount) {
    addEpisodeRecord(status.episode_index - 1, '✓ 已保存', true);
    lastSavedCount = status.saved;
  } else if (status.discarded > lastDiscardedCount) {
    addEpisodeRecord(status.episode_index - 1, '✕ 丢弃', false);
    lastDiscardedCount = status.discarded;
  }

  // 丢帧与延迟警告
  if (status.dropped_frames > 0) {
    primaryViewportEl.classList.add('drop-alert');
    osdFpsEl.textContent = `丢帧: ${status.dropped_frames}`;
    osdFpsEl.style.color = 'var(--bad)';
  } else {
    primaryViewportEl.classList.remove('drop-alert');
    osdFpsEl.textContent = '50.0 FPS';
    osdFpsEl.style.color = 'var(--text)';
  }

  const invalid = status.invalid_reason ? ` · 中止 ${status.invalid_reason}` : '';
  const saveErr = status.save_errors > 0 ? ` · 写入异常(${status.save_errors})` : '';
  statsEl.textContent = `帧: ${status.frames_recorded} · 已存: ${status.saved} · 丢弃: ${status.discarded}${invalid}${saveErr}`;
  if (status.save_errors > 0 && status.last_save_error) {
    showNotice(`落盘错误: ${status.last_save_error}`);
  }
  setControlAvailability(curState);

  // 遥测数据分发
  const telemetry = status.telemetry || {};
  const q = telemetry.state || [];
  if (q.length >= 22) {
    updateGroupBars(armJointsEl, q.slice(0, 6), -3.14, 3.14);
    updateGroupBars(handThumbEl, q.slice(6, 10), -1.5, 2.5);
    updateGroupBars(handIndexEl, q.slice(10, 14), -0.5, 2.0);
    updateGroupBars(handMiddleEl, q.slice(14, 18), -0.5, 2.0);
    updateGroupBars(handRingEl, q.slice(18, 22), -0.5, 2.0);
  }

  // 指尖受力字典
  const forces = telemetry.contact_forces || [0, 0, 0, 0];
  const forceDict = {
    index: forces[0] || 0,
    middle: forces[1] || 0,
    ring: forces[2] || 0,
    thumb: forces[3] || 0,
  };
  updateFingerRadar(forceDict);
}

function showNotice(message) {
  noticeEl.textContent = message;
  window.clearTimeout(showNotice.timeoutId);
  showNotice.timeoutId = window.setTimeout(() => {
    noticeEl.textContent = '';
  }, 3500);
}

async function act(name) {
  const body = {};
  if (name === 'label') {
    body.phase_label = phaseInput.value;
    body.language_instruction = instructionInput.value;
    osdInstructionEl.textContent = instructionInput.value;
  }
  try {
    const response = await fetch(`/api/action/${name}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    if (!response.ok) {
      const error = await response.json().catch(() => ({}));
      showNotice(error.error || `操作被拒绝: ${name}`);
    } else if (name === 'home') {
      showNotice('已一键复位至初始就绪姿态');
    }
  } catch (error) {
    showNotice('Hub 连接已断开');
  }
}

// 8. 按钮与全局键盘盲操体系
document.querySelectorAll('[data-act]').forEach((button) => {
  button.addEventListener('click', () => act(button.dataset.act));
});

// 全屏禅道模式切换
document.getElementById('btn-zen').addEventListener('click', () => {
  document.body.classList.toggle('zen-mode');
});

// 全键盘热键分发
document.addEventListener('keydown', (event) => {
  // 如果当前在输入框内，仅允许 Enter 触发更新
  if (event.target.closest('input, textarea')) {
    if (event.key === 'Enter') {
      event.preventDefault();
      act('label');
    }
    return;
  }

  const key = event.key.toLowerCase();

  // 1. 智能空格键 (Space): IDLE -> Start; RECORDING/PAUSED -> Stop; REVIEWING -> Save
  if (key === ' ' || event.code === 'Space') {
    event.preventDefault();
    if (lastState === 'IDLE') {
      act('start');
    } else if (lastState === 'RECORDING' || lastState === 'PAUSED') {
      act('stop');
    } else if (lastState === 'REVIEWING') {
      act('save');
    }
    return;
  }

  // 2. 试次审查决策键
  if (key === 's') {
    event.preventDefault();
    act('save');
    return;
  }
  if (key === 'd') {
    event.preventDefault();
    act('discard');
    return;
  }

  // 3. 数字键 1-4 阶段标记
  if (['1', '2', '3', '4'].includes(key)) {
    event.preventDefault();
    setPhaseByNum(parseInt(key, 10));
    return;
  }

  // 4. Tab 键主副视角无缝交换
  if (key === 'tab') {
    event.preventDefault();
    swapPrimaryCam();
    return;
  }

  // 5. F 键切换全屏禅道模式
  if (key === 'f') {
    event.preventDefault();
    document.body.classList.toggle('zen-mode');
    return;
  }

  // 6. H 键一键回初始姿态
  if (key === 'h') {
    event.preventDefault();
    act('home');
    return;
  }
});

// 9. 连接建立与 SSE 实时事件流
fetch('/api/status')
  .then((response) => response.json())
  .then(renderStatus)
  .catch(() => {
    stateEl.textContent = 'DISCONNECTED';
    stateEl.dataset.state = 'DISCONNECTED';
  });

const events = new EventSource('/events/telemetry');
events.onmessage = (event) => renderStatus(JSON.parse(event.data));
events.onerror = () => {
  stateEl.textContent = 'DISCONNECTED';
  stateEl.dataset.state = 'DISCONNECTED';
};
