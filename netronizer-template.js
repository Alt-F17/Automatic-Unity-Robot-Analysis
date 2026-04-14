function fmtInt(v) {
  return Number(v || 0).toLocaleString();
}

function setText(id, value) {
  const el = document.getElementById(id);
  if (el) {
    el.textContent = value;
  }
}

setText('model-title', DASHBOARD.model_name);
setText('model-subtitle', DASHBOARD.producer + ' · graph ' + DASHBOARD.graph_name + ' · opset ' + DASHBOARD.opset + ' · IR v' + DASHBOARD.ir_version);
setText('kpi-params', fmtInt(DASHBOARD.total_params));
setText('kpi-nodes', fmtInt(DASHBOARD.node_count));
setText('kpi-edges', fmtInt(DASHBOARD.edge_count));
setText('kpi-tensors', fmtInt(DASHBOARD.tensor_count));
setText('kpi-io', (DASHBOARD.input_header || DASHBOARD.input_count) + ' / ' + DASHBOARD.output_count);
setText('kpi-stages', fmtInt(DASHBOARD.stage_count));

setText('m-producer', DASHBOARD.producer);
setText('m-graph', DASHBOARD.graph_name);
setText('m-onnx', DASHBOARD.opset + ' / ' + DASHBOARD.ir_version);
setText('m-fan', DASHBOARD.max_fanin + ' / ' + DASHBOARD.max_fanout);
setText('m-fan-mean', DASHBOARD.mean_fanin + ' / ' + DASHBOARD.mean_fanout);
setText('m-branch', fmtInt(DASHBOARD.branchy_nodes));

const opList = document.getElementById('op-list');
(DASHBOARD.op_mix || []).slice(0, 14).forEach(function(item) {
  const card = document.createElement('div');
  card.className = 'op-item';
  card.innerHTML =
    '<div class="op-head">' +
      '<span class="op-name">' + item.op + '</span>' +
      '<span class="op-meta">' + fmtInt(item.count) + ' · ' + item.share + '%</span>' +
    '</div>' +
    '<div class="op-bar"><span style="width:' + Math.max(3, item.share) + '%"></span></div>';
  opList.appendChild(card);
});

const noteList = document.getElementById('note-list');
(DASHBOARD.notes || []).forEach(function(note) {
  const li = document.createElement('li');
  li.textContent = note;
  noteList.appendChild(li);
});

const ioRows = document.getElementById('io-table');
(DASHBOARD.inputs || []).forEach(function(item) {
  const tr = document.createElement('tr');
  tr.innerHTML = '<td>Input</td><td title="' + item.name + '">' + item.short + '</td><td>' + item.dtype + '</td><td>' + item.shape + '</td>';
  ioRows.appendChild(tr);
});
(DASHBOARD.outputs || []).forEach(function(item) {
  const tr = document.createElement('tr');
  tr.innerHTML = '<td>Output</td><td title="' + item.name + '">' + item.short + '</td><td>' + item.dtype + '</td><td>' + item.shape + '</td>';
  ioRows.appendChild(tr);
});

const heavyRows = document.getElementById('heavy-table');
(DASHBOARD.heavy_nodes || []).forEach(function(item) {
  const tr = document.createElement('tr');
  tr.innerHTML =
    '<td title="' + item.name + '">' + item.name + '</td>' +
    '<td>' + item.op + '</td>' +
    '<td>' + item.stage + '</td>' +
    '<td>' + fmtInt(item.params) + '</td>' +
    '<td>' + item.fanin + ' / ' + item.fanout + '</td>';
  heavyRows.appendChild(tr);
});

const frame = document.getElementById('netron-frame');
const frameWrap = document.getElementById('frame-wrap');
const status = document.getElementById('backend-status');
const direct = document.getElementById('netron-link');
const fullscreenButton = document.getElementById('netron-fullscreen');
frame.src = DASHBOARD.netron_url;
direct.href = DASHBOARD.netron_url;
direct.textContent = DASHBOARD.netron_url;
setText('backend-url', DASHBOARD.netron_url);

function syncFullscreenButton() {
  if (!fullscreenButton) {
    return;
  }
  const active = document.fullscreenElement === frameWrap;
  fullscreenButton.textContent = active ? 'Exit Fullscreen' : 'Fullscreen';
}

if (fullscreenButton && frameWrap && typeof frameWrap.requestFullscreen === 'function') {
  fullscreenButton.addEventListener('click', async function() {
    try {
      if (document.fullscreenElement === frameWrap) {
        await document.exitFullscreen();
      } else {
        await frameWrap.requestFullscreen();
      }
    } catch (err) {
      console.error('Fullscreen toggle failed', err);
    }
    syncFullscreenButton();
  });
  document.addEventListener('fullscreenchange', syncFullscreenButton);
} else if (fullscreenButton) {
  fullscreenButton.disabled = true;
  fullscreenButton.textContent = 'Fullscreen N/A';
}

frame.addEventListener('load', function() {
  status.textContent = 'Backend Synced';
  status.style.color = '#f0e68c';
  status.style.borderColor = 'rgba(240,230,140,0.5)';
  syncFullscreenButton();
});
