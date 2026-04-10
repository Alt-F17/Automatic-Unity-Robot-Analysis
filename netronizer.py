import argparse
import html
import importlib
import inspect
import json
import os
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from string import Template
from urllib.parse import urlparse

from vizzer import build_columns, extract_model


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Run a curated Netron-backed model intelligence frontend."
    )
    parser.add_argument("model", help="Path to a model file (for example: .onnx)")
    parser.add_argument("--host", default="127.0.0.1", help="Host for both frontend and backend")
    parser.add_argument("--port", type=int, default=8080, help="Frontend port")
    parser.add_argument(
        "--netron-port",
        type=int,
        default=None,
        help="Netron backend port (default: frontend port + 1)",
    )
    parser.add_argument(
        "--title",
        default="Netronizer",
        help="Title shown in the frontend header",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Start servers without auto-opening a browser tab",
    )
    return parser


def _shape_text(shape):
    if not shape:
        return "?"
    return "x".join(str(x) for x in shape)


def _shape_example_with_batch_one(shape):
    if not shape:
        return "?"

    base_dims = []
    example_dims = []
    has_symbolic = False

    for dim in shape:
        if isinstance(dim, int) and dim > 0:
            text = str(dim)
            base_dims.append(text)
            example_dims.append(text)
            continue

        dim_text = str(dim) if dim not in (None, "") else "?"
        base_dims.append(dim_text)
        example_dims.append("1")
        has_symbolic = True

    if not has_symbolic:
      return "x".join(base_dims)

    return "x".join(example_dims)


def _short_name(name):
    if not name:
        return "(anonymous)"
    parts = str(name).split("/")
    return parts[-1] or name


def _netron_kwargs(netron_start, host, port):
    sig = inspect.signature(netron_start)
    params = sig.parameters
    kwargs = {"browse": False} if "browse" in params else {}

    if "address" in params:
        kwargs["address"] = (host, port)
    else:
        if "host" in params:
            kwargs["host"] = host
        if "port" in params:
            kwargs["port"] = port

    return kwargs


def _augment_layout_metrics(model_data):
    nodes = model_data["nodes"]
    inputs = model_data["inputs"]
    outputs = model_data["outputs"]

    columns, node_col, stage_meta, node_branches = build_columns(nodes, inputs, outputs)
    for node in nodes:
        nid = node["id"]
        branch_ids = node_branches.get(nid, [])
        node["metrics"]["depth_hint"] = int(node_col.get(nid, 0))
        node["metrics"]["stage_hint"] = int(node_col.get(nid, 0))
        node["metrics"]["branch_count"] = len(branch_ids)
        node["metrics"]["branch_ids"] = branch_ids[:8]

    return columns, stage_meta


def _build_dashboard(model_data, model_path, netron_url):
    meta = model_data["meta"]
    nodes = model_data["nodes"]
    inputs = model_data["inputs"]
    outputs = model_data["outputs"]

    _augment_layout_metrics(model_data)

    sorted_ops = sorted(meta["op_counts"].items(), key=lambda item: (-item[1], item[0]))
    op_total = max(1, sum(count for _, count in sorted_ops))
    op_mix = [
        {
            "op": op,
            "count": count,
            "share": round((count / op_total) * 100, 2),
        }
        for op, count in sorted_ops
    ]

    heavy_nodes = []
    for node in nodes:
        weight_params = int(node.get("metrics", {}).get("weight_params", 0))
        if weight_params <= 0:
            continue
        heavy_nodes.append(
            {
                "id": node["id"],
                "op": node["op"],
                "name": _short_name(node.get("name") or node["op"]),
                "params": weight_params,
                "stage": int(node.get("metrics", {}).get("stage_hint", 0)),
                "fanin": int(node.get("metrics", {}).get("fanin", 0)),
                "fanout": int(node.get("metrics", {}).get("fanout", 0)),
            }
        )

    heavy_nodes.sort(key=lambda item: (-item["params"], item["id"]))
    heavy_nodes = heavy_nodes[:14]

    input_specs = [
        {
            "name": item["name"],
            "short": _short_name(item["name"]),
            "dtype": item.get("dtype", "?"),
            "shape": _shape_text(item.get("shape", ["?"])),
        }
        for item in inputs
    ]

    output_specs = [
        {
            "name": item["name"],
            "short": _short_name(item["name"]),
            "dtype": item.get("dtype", "?"),
            "shape": _shape_text(item.get("shape", ["?"])),
        }
        for item in outputs
    ]

    input_header = str(len(input_specs))
    if input_specs:
        batch_example = _shape_example_with_batch_one(inputs[0].get("shape", ["?"]))
        input_header = f"{batch_example}"

    branchy_nodes = sum(1 for node in nodes if int(node.get("metrics", {}).get("branch_count", 0)) > 1)
    mean_fanin = round(sum(int(node["metrics"].get("fanin", 0)) for node in nodes) / max(1, len(nodes)), 2)
    mean_fanout = round(sum(int(node["metrics"].get("fanout", 0)) for node in nodes) / max(1, len(nodes)), 2)
    max_stage = max((int(node.get("metrics", {}).get("stage_hint", 0)) for node in nodes), default=0) + 1

    top3_param_share = 0.0
    if meta.get("total_params", 0) > 0 and heavy_nodes:
        top3 = sum(item["params"] for item in heavy_nodes[:3])
        top3_param_share = round((top3 / meta["total_params"]) * 100, 1)

    notes = []
    if op_mix:
        dominant = op_mix[0]
        notes.append(
            f"Dominant operator family: {dominant['op']} at {dominant['share']}% of all node ops."
        )
    if top3_param_share > 0:
        notes.append(
            f"Parameter concentration: top 3 param-heavy nodes carry {top3_param_share}% of model parameters."
        )
    if branchy_nodes > 0:
        notes.append(
            f"Branching complexity: {branchy_nodes} nodes feed multiple output lanes."
        )
    if int(meta.get("max_fanin", 0)) >= 3:
        notes.append(
            f"High dependency joins detected (max fan-in = {int(meta['max_fanin'])}); inspect merge points for instability."
        )
    if int(meta.get("max_fanout", 0)) >= 4:
        notes.append(
            f"Strong broadcast behavior detected (max fan-out = {int(meta['max_fanout'])}); these nodes are graph-critical."
        )

    if not notes:
        notes.append("Topology is compact and mostly linear, with low fan-in / fan-out coupling.")

    return {
        "model_name": os.path.basename(model_path),
        "graph_name": meta.get("graph_name", "unnamed_graph"),
        "producer": meta.get("producer") or "unknown",
        "opset": ", ".join(str(v) for v in meta.get("opset", [])) or "unknown",
        "ir_version": meta.get("ir_version", "?"),
        "node_count": int(meta.get("node_count", len(nodes))),
        "edge_count": int(meta.get("edge_count", len(model_data.get("edges", [])))),
        "tensor_count": int(meta.get("tensor_count", len(model_data.get("tensors", [])))),
        "initializer_count": int(meta.get("initializer_count", len(model_data.get("weights", {})))),
        "input_count": len(input_specs),
        "output_count": len(output_specs),
        "input_header": input_header,
        "total_params": int(meta.get("total_params", 0)),
        "max_fanin": int(meta.get("max_fanin", 0)),
        "max_fanout": int(meta.get("max_fanout", 0)),
        "mean_fanin": mean_fanin,
        "mean_fanout": mean_fanout,
        "stage_count": max_stage,
        "branchy_nodes": branchy_nodes,
        "inputs": input_specs,
        "outputs": output_specs,
        "op_mix": op_mix,
        "heavy_nodes": heavy_nodes,
        "notes": notes,
        "netron_url": netron_url,
    }


def _html_template():
    return Template(
        """<!DOCTYPE html>
<html lang=\"en\">
<head>
<meta charset=\"UTF-8\">
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
<title>$PAGE_TITLE</title>
<link rel=\"preconnect\" href=\"https://fonts.googleapis.com\">
<link rel=\"preconnect\" href=\"https://fonts.gstatic.com\" crossorigin>
<link href=\"https://fonts.googleapis.com/css2?family=Manrope:wght@400;500;600;700;800&family=IBM+Plex+Mono:wght@400;500&display=swap\" rel=\"stylesheet\">
<style>
:root {
  --bg: #0a0a0a;
  --panel: #0c0c0c;
  --panel-2: #101010;
  --line: #1b1b1b;
  --txt: #dddddd;
  --muted: #7a7a7a;
  --accent: #f0e68c;
  --accent-2: #b7b17a;
  --danger: #c0392b;
  --shadow: 0 8px 22px rgba(0,0,0,0.28);
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  font-family: 'Courier New', monospace;
  color: var(--txt);
  background:
    radial-gradient(1200px 700px at 6% -8%, rgba(240, 230, 140, 0.06), transparent 62%),
    radial-gradient(900px 540px at 100% 0%, rgba(192, 57, 43, 0.05), transparent 58%),
    var(--bg);
  min-height: 100vh;
}
.wrapper {
  width: min(1560px, 95vw);
  margin: 28px auto 36px;
}
.topbar {
  border: 1px solid var(--line);
  border-radius: 3px;
  background: linear-gradient(180deg, rgba(14,14,14,0.96), rgba(10,10,10,0.98));
  box-shadow: var(--shadow);
  padding: 20px 24px;
  margin-bottom: 18px;
}
.topline {
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 16px;
  flex-wrap: wrap;
}
.badge {
  display: inline-flex;
  align-items: center;
  gap: 8px;
  font-size: 11px;
  letter-spacing: 1.4px;
  text-transform: uppercase;
  color: var(--accent);
}
.badge-dot {
  width: 8px;
  height: 8px;
  border-radius: 999px;
  background: linear-gradient(120deg, var(--accent), #9f9960);
  box-shadow: 0 0 12px rgba(240, 230, 140, 0.35);
}
.title {
  font-size: clamp(18px, 2.4vw, 28px);
  font-weight: 700;
  letter-spacing: 0.8px;
}
.subtitle {
  margin-top: 7px;
  font-size: 11px;
  color: var(--muted);
  font-family: 'IBM Plex Mono', monospace;
}
.kpi-grid {
  margin-top: 16px;
  display: grid;
  grid-template-columns: repeat(6, minmax(0, 1fr));
  gap: 10px;
}
.kpi {
  border: 1px solid var(--line);
  border-radius: 2px;
  background: rgba(16, 16, 16, 0.9);
  padding: 10px 12px;
}
.kpi label {
  font-size: 10px;
  color: var(--muted);
  letter-spacing: 1px;
  text-transform: uppercase;
}
.kpi .value {
  margin-top: 6px;
  font-size: 19px;
  font-weight: 700;
  color: var(--accent);
}
.main {
  display: grid;
  grid-template-columns: 1.05fr 1.55fr;
  gap: 14px;
}
.panel {
  border: 1px solid var(--line);
  border-radius: 3px;
  background: linear-gradient(180deg, rgba(14,14,14,0.96), rgba(10,10,10,0.98));
  box-shadow: var(--shadow);
}
.section {
  padding: 16px 18px;
  border-bottom: 1px solid rgba(255,255,255,0.05);
}
.section:last-child { border-bottom: none; }
.h2 {
  font-size: 11px;
  text-transform: uppercase;
  letter-spacing: 1.3px;
  color: var(--accent-2);
  margin-bottom: 10px;
}
.metric-row {
  display: flex;
  justify-content: space-between;
  gap: 12px;
  padding: 8px 0;
  border-bottom: 1px dashed rgba(255,255,255,0.08);
  font-size: 12px;
}
.metric-row:last-child { border-bottom: none; }
.metric-row .k { color: var(--muted); }
.metric-row .v { font-family: 'IBM Plex Mono', monospace; }
.op-list {
  display: grid;
  gap: 8px;
}
.op-item {
  border: 1px solid rgba(240,230,140,0.18);
  background: rgba(12,12,12,0.86);
  border-radius: 2px;
  padding: 8px 10px;
}
.op-head {
  display: flex;
  justify-content: space-between;
  align-items: center;
  font-size: 12px;
  margin-bottom: 6px;
}
.op-head .op-name { font-weight: 700; color: #d7d7d7; }
.op-head .op-meta { color: var(--muted); font-family: 'IBM Plex Mono', monospace; }
.op-bar {
  height: 6px;
  border-radius: 1px;
  background: rgba(240,230,140,0.13);
  overflow: hidden;
}
.op-bar > span {
  display: block;
  height: 100%;
  border-radius: inherit;
  background: linear-gradient(90deg, var(--accent), #9a935f);
}
.note-list {
  display: grid;
  gap: 8px;
  list-style: none;
}
.note-list li {
  border: 1px solid rgba(192,57,43,0.22);
  background: rgba(18,10,10,0.62);
  border-radius: 2px;
  padding: 8px 10px;
  font-size: 12px;
  color: #c3b8b8;
  line-height: 1.45;
}
.table-wrap {
  border: 1px solid var(--line);
  border-radius: 2px;
  overflow: hidden;
  background: rgba(10,10,10,0.86);
}
table {
  width: 100%;
  border-collapse: collapse;
  font-size: 12px;
}
th, td {
  padding: 8px 10px;
  border-bottom: 1px solid rgba(255,255,255,0.06);
  text-align: left;
}
th {
  color: #b7b17a;
  font-size: 10px;
  letter-spacing: 1px;
  text-transform: uppercase;
  background: rgba(15,15,15,0.95);
}
tr:last-child td { border-bottom: none; }
.small {
  color: var(--muted);
  font-family: 'IBM Plex Mono', monospace;
  font-size: 11px;
}
.right-top {
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 12px;
  margin-bottom: 10px;
  flex-wrap: wrap;
}
.quick-actions {
  display: flex;
  align-items: center;
  gap: 8px;
}
.status {
  border: 1px solid rgba(240,230,140,0.45);
  border-radius: 2px;
  padding: 4px 10px;
  font-size: 10px;
  letter-spacing: 1px;
  text-transform: uppercase;
  color: var(--accent);
  font-family: 'IBM Plex Mono', monospace;
}
.control-btn {
  border: 1px solid rgba(240,230,140,0.45);
  background: rgba(18,18,18,0.95);
  color: var(--accent);
  font-family: 'IBM Plex Mono', monospace;
  font-size: 10px;
  letter-spacing: 0.9px;
  text-transform: uppercase;
  border-radius: 2px;
  padding: 5px 10px;
  cursor: pointer;
}
.control-btn:hover {
  background: rgba(240,230,140,0.12);
}
.frame-wrap {
  border: 1px solid var(--line);
  border-radius: 2px;
  overflow: hidden;
  height: min(76vh, 980px);
  min-height: 520px;
  background: #090909;
}
iframe {
  width: 100%;
  height: 100%;
  border: 0;
  background: #090909;
}
.footer {
  margin-top: 12px;
  color: var(--muted);
  font-size: 11px;
  font-family: 'IBM Plex Mono', monospace;
}
@keyframes rise {
  from { opacity: 0; transform: translateY(6px); }
  to { opacity: 1; transform: translateY(0); }
}
.panel, .topbar { animation: rise .35s ease-out; }
@media (max-width: 1220px) {
  .kpi-grid { grid-template-columns: repeat(3, minmax(0, 1fr)); }
  .main { grid-template-columns: 1fr; }
  .frame-wrap { min-height: 420px; }
}
@media (max-width: 760px) {
  .wrapper { width: 96vw; margin-top: 14px; }
  .kpi-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .topbar { padding: 14px; }
  .section { padding: 12px; }
}
</style>
</head>
<body>
<div class=\"wrapper\">
  <section class=\"topbar\">
    <div class=\"topline\">
      <div>
        <div class=\"badge\"><span class=\"badge-dot\"></span>$APP_TITLE · Netron Backend Active</div>
        <div class=\"title\" id=\"model-title\"></div>
        <div class=\"subtitle\" id=\"model-subtitle\"></div>
      </div>
      <div class=\"status\" id=\"backend-status\">Connecting Backend</div>
    </div>
    <div class=\"kpi-grid\">
      <div class=\"kpi\"><label>Parameters</label><div class=\"value\" id=\"kpi-params\">-</div></div>
      <div class=\"kpi\"><label>Nodes</label><div class=\"value\" id=\"kpi-nodes\">-</div></div>
      <div class=\"kpi\"><label>Edges</label><div class=\"value\" id=\"kpi-edges\">-</div></div>
      <div class=\"kpi\"><label>Tensors</label><div class=\"value\" id=\"kpi-tensors\">-</div></div>
      <div class=\"kpi\"><label>Inputs / Outputs</label><div class=\"value\" id=\"kpi-io\">-</div></div>
      <div class=\"kpi\"><label>Stages</label><div class=\"value\" id=\"kpi-stages\">-</div></div>
    </div>
  </section>

  <section class=\"main\">
    <div class=\"panel\">
      <div class=\"section\">
        <div class=\"h2\">Topology Signals</div>
        <div class=\"metric-row\"><span class=\"k\">Producer</span><span class=\"v\" id=\"m-producer\"></span></div>
        <div class=\"metric-row\"><span class=\"k\">Graph</span><span class=\"v\" id=\"m-graph\"></span></div>
        <div class=\"metric-row\"><span class=\"k\">ONNX Opset / IR</span><span class=\"v\" id=\"m-onnx\"></span></div>
        <div class=\"metric-row\"><span class=\"k\">Max Fan-In / Fan-Out</span><span class=\"v\" id=\"m-fan\"></span></div>
        <div class=\"metric-row\"><span class=\"k\">Mean Fan-In / Fan-Out</span><span class=\"v\" id=\"m-fan-mean\"></span></div>
        <div class=\"metric-row\"><span class=\"k\">Branch-Critical Nodes</span><span class=\"v\" id=\"m-branch\"></span></div>
      </div>

      <div class=\"section\">
        <div class=\"h2\">Operator Distribution</div>
        <div class=\"op-list\" id=\"op-list\"></div>
      </div>

      <div class=\"section\">
        <div class=\"h2\">Architecture Notes</div>
        <ul class=\"note-list\" id=\"note-list\"></ul>
      </div>

      <div class=\"section\">
        <div class=\"h2\">Model Boundaries</div>
        <div class=\"table-wrap\">
          <table>
            <thead><tr><th>Direction</th><th>Name</th><th>DType</th><th>Shape</th></tr></thead>
            <tbody id=\"io-table\"></tbody>
          </table>
        </div>
      </div>
    </div>

    <div class=\"panel\">
      <div class=\"section\">
        <div class=\"right-top\">
          <div>
            <div class=\"h2\">Netron Surface</div>
            <div class=\"small\">Live backend visualization with curated telemetry on the left.</div>
          </div>
          <div class=\"quick-actions\">
            <a class=\"small\" id=\"netron-link\" target=\"_blank\" rel=\"noopener\">Open Netron Direct</a>
            <button type=\"button\" class=\"control-btn\" id=\"netron-fullscreen\">Fullscreen</button>
          </div>
        </div>
        <div class=\"frame-wrap\" id=\"frame-wrap\">
          <iframe id=\"netron-frame\" title=\"Netron Backend\" loading=\"eager\"></iframe>
        </div>
      </div>

      <div class=\"section\">
        <div class=\"h2\">Parameter Hotspots</div>
        <div class=\"table-wrap\">
          <table>
            <thead><tr><th>Node</th><th>Op</th><th>Stage</th><th>Params</th><th>Fan</th></tr></thead>
            <tbody id=\"heavy-table\"></tbody>
          </table>
        </div>
        <div class=\"footer\">Backend endpoint: <span id=\"backend-url\"></span></div>
      </div>
    </div>
  </section>
</div>

<script>
const DASHBOARD = $DASHBOARD_JSON;

function fmtInt(v) {
  return Number(v || 0).toLocaleString();
}

function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
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
  if (!fullscreenButton) return;
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
</script>
</body>
</html>
"""
    )


def _build_frontend_html(app_title, dashboard):
    template = _html_template()
    return template.safe_substitute(
        PAGE_TITLE=html.escape(f"{app_title} - {dashboard['model_name']}") ,
        APP_TITLE=html.escape(app_title),
        DASHBOARD_JSON=json.dumps(dashboard, separators=(",", ":")),
    )


def _make_handler(document, dashboard_json):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                body = document.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == "/api/dashboard.json":
                body = dashboard_json.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == "/healthz":
                body = b"ok"
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            self.send_response(404)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"Not found")

        def log_message(self, fmt, *args):
            print(f"[frontend] {self.address_string()} - {fmt % args}")

    return Handler


def main():
    parser = _build_parser()
    args = parser.parse_args()

    model_path = os.path.abspath(args.model)
    if not os.path.isfile(model_path):
        print(f"Error: model not found: {model_path}")
        return 1

    netron_port = args.netron_port if args.netron_port is not None else args.port + 1
    if netron_port == args.port:
        print("Error: --port and --netron-port must be different.")
        return 1

    try:
        netron = importlib.import_module("netron")
    except ImportError:
        print("Error: Netron is not installed in this environment.")
        print("Install it with: pip install netron")
        return 1

    netron_url = f"http://{args.host}:{netron_port}"
    frontend_url = f"http://{args.host}:{args.port}"

    print(f"Loading model telemetry: {model_path}")
    model_data = extract_model(model_path)
    dashboard = _build_dashboard(model_data, model_path, netron_url)
    document = _build_frontend_html(args.title, dashboard)
    dashboard_json = json.dumps(dashboard, separators=(",", ":"))

    kwargs = _netron_kwargs(netron.start, args.host, netron_port)

    print(f"Starting Netron backend on {netron_url}")
    try:
        netron.start(model_path, **kwargs)
    except Exception as exc:
        print(f"Error: failed to start Netron backend: {exc}")
        return 1

    handler = _make_handler(document, dashboard_json)
    try:
        frontend_server = ThreadingHTTPServer((args.host, args.port), handler)
    except OSError as exc:
        print(f"Error: failed to start frontend server on {frontend_url}: {exc}")
        try:
            netron.stop()
        except Exception:
            pass
        return 1

    print(f"Starting curated frontend on {frontend_url}")
    print(f"Netron backend endpoint: {netron_url}")
    print("Press Ctrl+C to stop both servers.")

    if not args.no_browser:
        webbrowser.open(frontend_url)

    try:
        frontend_server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping services...")
    finally:
        frontend_server.shutdown()
        frontend_server.server_close()
        try:
            netron.stop()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
