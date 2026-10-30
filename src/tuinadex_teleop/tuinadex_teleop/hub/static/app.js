const stateEl = document.getElementById('state');
const statsEl = document.getElementById('stats');
const jointsEl = document.getElementById('joints');
const forcesEl = document.getElementById('forces');
const episodeEl = document.getElementById('episode');
const labelSummaryEl = document.getElementById('label-summary');
const noticeEl = document.getElementById('notice');
const FORCE_NAMES = ['index', 'middle', 'ring', 'thumb'];

function makeRow(label) {
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

for (let i = 0; i < 22; i += 1) {
  jointsEl.appendChild(makeRow(`J${String(i).padStart(2, '0')}`));
}
FORCE_NAMES.forEach((name) => forcesEl.appendChild(makeRow(name)));

function updateBars(container, values, lower, upper) {
  const rows = container.children;
  for (let i = 0; i < rows.length && i < values.length; i += 1) {
    const value = Number(values[i]);
    if (!Number.isFinite(value)) continue;
    const ratio = Math.max(0, Math.min(1, (value - lower) / (upper - lower)));
    rows[i].querySelector('.bar-fill').style.width = `${(ratio * 100).toFixed(1)}%`;
    rows[i].querySelector('.bar-value').textContent = value.toFixed(2);
  }
}

function setControlAvailability(state) {
  const allowed = {
    IDLE: new Set(['start']),
    RECORDING: new Set(['pause', 'stop']),
    PAUSED: new Set(['resume', 'stop']),
    REVIEWING: new Set(['save', 'discard']),
    SAVING: new Set(),
  };
  document.querySelectorAll('[data-act]').forEach((button) => {
    if (button.dataset.act === 'label') return;
    button.disabled = !(allowed[state] || new Set()).has(button.dataset.act);
  });
}

function renderStatus(status) {
  stateEl.textContent = status.state;
  stateEl.dataset.state = status.state;
  episodeEl.textContent = `EP ${String(status.episode_index || 0).padStart(2, '0')}`;
  labelSummaryEl.textContent = status.phase_label || 'idle';
  const invalid = status.invalid_reason ? ` · 中止 ${status.invalid_reason}` : '';
  statsEl.textContent = `帧 ${status.frames_recorded} · 已存 ${status.saved} · 丢弃 ${status.discarded} · 丢帧 ${status.dropped_frames}${invalid}`;
  setControlAvailability(status.state);

  const telemetry = status.telemetry || {};
  updateBars(jointsEl, telemetry.state || [], -0.3, 2.0);
  updateBars(forcesEl, telemetry.contact_forces || [], 0, 10);
}

function showNotice(message) {
  noticeEl.textContent = message;
  window.clearTimeout(showNotice.timeoutId);
  showNotice.timeoutId = window.setTimeout(() => {
    noticeEl.textContent = '';
  }, 3000);
}

async function act(name) {
  const body = {};
  if (name === 'label') {
    body.phase_label = document.getElementById('phase').value;
    body.language_instruction = document.getElementById('instruction').value;
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
    }
  } catch (error) {
    showNotice('Hub 连接已断开');
  }
}

document.querySelectorAll('[data-act]').forEach((button) => {
  button.addEventListener('click', () => act(button.dataset.act));
});

const shortcuts = {
  ' ': 'start',
  p: 'pause',
  r: 'resume',
  t: 'stop',
  s: 'save',
  d: 'discard',
};

document.addEventListener('keydown', (event) => {
  if (event.target.closest('input, textarea, select, button')) return;
  const action = shortcuts[event.key.toLowerCase()];
  if (!action) return;
  const button = document.querySelector(`[data-act="${action}"]`);
  if (button && button.disabled) return;
  event.preventDefault();
  act(action);
});

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
