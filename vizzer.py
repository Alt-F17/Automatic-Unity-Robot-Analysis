import json
import os
import sys
import html as html_lib
from collections import deque

import onnx
import numpy as np
from onnx import AttributeProto, TensorProto, numpy_helper


def _shape_dtype_from_type(type_proto):
    try:
        if not type_proto or not type_proto.HasField("tensor_type"):
            return ["?"], "?"
        tt = type_proto.tensor_type
        shape = []
        for dim in tt.shape.dim:
            if dim.dim_value > 0:
                shape.append(int(dim.dim_value))
            elif dim.dim_param:
                shape.append(dim.dim_param)
            else:
                shape.append("?")
        dtype = TensorProto.DataType.Name(tt.elem_type) if tt.elem_type else "?"
        return shape or ["?"], dtype or "?"
    except Exception:
        return ["?"], "?"


def _shape_dtype_from_initializer(initializer):
    try:
        shape = list(initializer.dims) or ["?"]
        dtype = TensorProto.DataType.Name(initializer.data_type) if initializer.data_type else "?"
        return shape, dtype
    except Exception:
        return ["?"], "?"


def _shape_text(shape):
    if not shape:
        return "?"
    return "x".join(str(x) for x in shape)


def _clip_list(values, max_items=8):
    values = list(values)
    if len(values) <= max_items:
        return values
    return values[:max_items] + [f"...(+{len(values) - max_items})"]


def _parse_attribute(attr):
    try:
        if attr.type == AttributeProto.FLOAT:
            return round(float(attr.f), 6)
        if attr.type == AttributeProto.INT:
            return int(attr.i)
        if attr.type == AttributeProto.STRING:
            return attr.s.decode("utf-8", "ignore")
        if attr.type == AttributeProto.FLOATS:
            return _clip_list([round(float(v), 6) for v in attr.floats])
        if attr.type == AttributeProto.INTS:
            return _clip_list([int(v) for v in attr.ints])
        if attr.type == AttributeProto.STRINGS:
            return _clip_list([v.decode("utf-8", "ignore") for v in attr.strings])
        if attr.type == AttributeProto.TENSOR:
            arr = numpy_helper.to_array(attr.t)
            info = {
                "dtype": str(arr.dtype),
                "shape": list(arr.shape),
                "params": int(arr.size),
            }
            if arr.size > 0 and np.issubdtype(arr.dtype, np.number):
                info["min"] = round(float(np.min(arr)), 5)
                info["max"] = round(float(np.max(arr)), 5)
            return info
        if attr.type == AttributeProto.GRAPH:
            return f"<graph:{attr.g.name or 'anonymous'}>"
        if attr.type == AttributeProto.GRAPHS:
            return _clip_list([f"<graph:{g.name or 'anonymous'}>" for g in attr.graphs], max_items=4)
    except Exception:
        return "<unparsed>"
    return "<unsupported>"


def _upsert_tensor(tensors, name, shape=None, dtype=None, role=None):
    if not name:
        return
    info = tensors.setdefault(name, {"name": name, "shape": ["?"], "dtype": "?", "roles": set()})
    if shape and (info["shape"] == ["?"] or ("?" in info["shape"] and "?" not in shape)):
        info["shape"] = list(shape)
    if dtype and (info["dtype"] in ("", "?") and dtype not in ("", "?")):
        info["dtype"] = dtype
    if role:
        info["roles"].add(role)


def extract_model(path):
    model = onnx.load(path)
    graph = model.graph

    tensors = {}

    inputs = []
    for inp in graph.input:
        shape, dtype = _shape_dtype_from_type(inp.type)
        inputs.append({"name": inp.name, "shape": shape, "dtype": dtype})
        _upsert_tensor(tensors, inp.name, shape, dtype, "input")

    outputs = []
    for out in graph.output:
        shape, dtype = _shape_dtype_from_type(out.type)
        outputs.append({"name": out.name, "shape": shape, "dtype": dtype})
        _upsert_tensor(tensors, out.name, shape, dtype, "output")

    for vi in graph.value_info:
        shape, dtype = _shape_dtype_from_type(vi.type)
        _upsert_tensor(tensors, vi.name, shape, dtype, "value_info")

    weights = {}
    total_params = 0
    for init in graph.initializer:
        init_shape, init_dtype = _shape_dtype_from_initializer(init)
        _upsert_tensor(tensors, init.name, init_shape, init_dtype, "initializer")

        try:
            arr = numpy_helper.to_array(init)
            params = int(arr.size)
            total_params += params

            weight_info = {
                "shape": list(arr.shape),
                "dtype": str(arr.dtype),
                "params": params,
            }

            if arr.size > 0 and np.issubdtype(arr.dtype, np.number):
                flat = arr.astype(np.float64, copy=False).flatten()
                hist, edges = np.histogram(flat, bins=16)
                weight_info.update({
                    "mean": round(float(np.mean(arr)), 5),
                    "std": round(float(np.std(arr)), 5),
                    "min": round(float(np.min(arr)), 5),
                    "max": round(float(np.max(arr)), 5),
                    "sparsity": round(float(np.mean(np.abs(arr) < 0.01)), 4),
                    "hist": [int(x) for x in hist],
                    "hist_edges": [round(float(x), 4) for x in edges],
                })

            weights[init.name] = weight_info
        except Exception:
            weights[init.name] = {
                "shape": init_shape,
                "dtype": init_dtype,
                "params": 0,
            }

    nodes = []
    op_counts = {}
    for i, node in enumerate(graph.node):
        op = node.op_type or "Unknown"
        op_counts[op] = op_counts.get(op, 0) + 1

        attrs = {attr.name: _parse_attribute(attr) for attr in node.attribute}
        weight_names = [inp_name for inp_name in node.input if inp_name in weights]

        nodes.append({
            "id": i,
            "op": op,
            "domain": node.domain or "",
            "name": node.name,
            "inputs": list(node.input),
            "outputs": list(node.output),
            "attrs": attrs,
            "weight_names": weight_names,
            "metrics": {},
            "io": {"inputs": [], "outputs": []},
        })

    tensor_producer = {}
    tensor_consumers = {}
    for node in nodes:
        for out_name in node["outputs"]:
            if out_name:
                tensor_producer[out_name] = node["id"]
        for inp_name in node["inputs"]:
            if inp_name:
                tensor_consumers.setdefault(inp_name, []).append(node["id"])

    deps = {node["id"]: set() for node in nodes}
    rev_deps = {node["id"]: set() for node in nodes}
    edge_map = {}
    for node in nodes:
        nid = node["id"]
        for inp_name in node["inputs"]:
            src = tensor_producer.get(inp_name)
            if src is None or src == nid:
                continue
            deps[nid].add(src)
            rev_deps[src].add(nid)

            key = (src, nid)
            if key not in edge_map:
                edge_map[key] = {"from": src, "to": nid, "count": 0, "tensors": []}
            edge_map[key]["count"] += 1
            if inp_name and len(edge_map[key]["tensors"]) < 4:
                edge_map[key]["tensors"].append(inp_name)

    edges = [edge_map[key] for key in sorted(edge_map)]

    for node in nodes:
        nid = node["id"]
        weight_params = int(sum(weights[name]["params"] for name in node["weight_names"]))
        node["metrics"] = {
            "fanin": len(deps[nid]),
            "fanout": len(rev_deps[nid]),
            "input_count": len([name for name in node["inputs"] if name]),
            "output_count": len([name for name in node["outputs"] if name]),
            "weight_tensors": len(node["weight_names"]),
            "weight_params": weight_params,
            "depth_hint": 0,
        }
        node["io"] = {
            "inputs": [
                {
                    "name": name,
                    "shape": tensors.get(name, {}).get("shape", ["?"]),
                    "dtype": tensors.get(name, {}).get("dtype", "?"),
                }
                for name in node["inputs"] if name
            ][:8],
            "outputs": [
                {
                    "name": name,
                    "shape": tensors.get(name, {}).get("shape", ["?"]),
                    "dtype": tensors.get(name, {}).get("dtype", "?"),
                }
                for name in node["outputs"] if name
            ][:8],
        }

    tensor_list = []
    for name, spec in tensors.items():
        tensor_list.append({
            "name": name,
            "shape": spec.get("shape", ["?"]),
            "dtype": spec.get("dtype", "?"),
            "roles": sorted(spec.get("roles", [])),
            "consumers": len(tensor_consumers.get(name, [])),
        })
    tensor_list.sort(key=lambda item: item["name"])

    meta = {
        "filename": os.path.basename(path),
        "graph_name": graph.name or "unnamed_graph",
        "ir_version": model.ir_version,
        "opset": [opset.version for opset in model.opset_import],
        "producer": f"{model.producer_name} {model.producer_version}".strip(),
        "total_params": total_params,
        "op_counts": op_counts,
        "node_count": len(nodes),
        "edge_count": len(edges),
        "initializer_count": len(graph.initializer),
        "tensor_count": len(tensor_list),
        "max_fanin": max((len(v) for v in deps.values()), default=0),
        "max_fanout": max((len(v) for v in rev_deps.values()), default=0),
    }

    return {
        "meta": meta,
        "inputs": inputs,
        "outputs": outputs,
        "nodes": nodes,
        "edges": edges,
        "weights": weights,
        "tensors": tensor_list,
    }


def build_columns(nodes, inputs, outputs):
    """
    Build a feed-forward layout with branch lanes:
    - stage assignment by dependency depth (with optional compression)
    - auxiliary constants shifted near the stage where they are consumed
    - output-ancestry lanes to keep branches visually grouped
    - barycentric row ordering within each stage to reduce crossings
    """
    del inputs

    node_lookup = {node["id"]: node for node in nodes}

    tensor_producer = {}
    for node in nodes:
        for out_name in node["outputs"]:
            if out_name:
                tensor_producer[out_name] = node["id"]

    deps = {node["id"]: set() for node in nodes}
    succ = {node["id"]: set() for node in nodes}
    for node in nodes:
        nid = node["id"]
        for inp_name in node["inputs"]:
            src = tensor_producer.get(inp_name)
            if src is None or src == nid:
                continue
            deps[nid].add(src)
            succ[src].add(nid)

    indegree = {nid: len(parents) for nid, parents in deps.items()}
    depth = {}
    topo = []
    queue = deque(sorted([nid for nid, degree in indegree.items() if degree == 0]))
    while queue:
        nid = queue.popleft()
        topo.append(nid)
        depth[nid] = max((depth[parent] for parent in deps[nid] if parent in depth), default=-1) + 1
        for child in sorted(succ[nid]):
            indegree[child] -= 1
            if indegree[child] == 0:
                queue.append(child)

    for nid in sorted(deps):
        if nid not in depth:
            depth[nid] = max((depth.get(parent, 0) for parent in deps[nid]), default=0)
            topo.append(nid)

    output_names = [out.get("name") for out in outputs if isinstance(out, dict)]
    node_branches = {nid: set() for nid in node_lookup}

    for out_idx, out_name in enumerate(output_names):
        if not out_name:
            continue
        root = tensor_producer.get(out_name)
        if root is None:
            continue

        stack = [root]
        seen = set()
        while stack:
            current = stack.pop()
            if current in seen:
                continue
            seen.add(current)
            node_branches[current].add(out_idx)
            stack.extend(deps[current])

    for _ in range(2):
        changed = False
        for nid in reversed(topo):
            if node_branches[nid]:
                continue
            inherited = set()
            for child in succ[nid]:
                inherited.update(node_branches[child])
            if inherited:
                node_branches[nid] = inherited
                changed = True
        if not changed:
            break

    max_depth = max(depth.values(), default=0)
    if max_depth <= 12:
        stage_raw = {nid: depth[nid] for nid in depth}
    else:
        target_stages = max(8, min(12, int(round((len(nodes) ** 0.5) * 1.8))))
        target_stages = max(target_stages, 2)
        stage_raw = {}
        for nid, d in depth.items():
            stage_raw[nid] = int(round((d / max_depth) * (target_stages - 1)))

    def is_aux_node(nid):
        node = node_lookup[nid]
        if node["op"] == "Constant":
            return True
        return (
            node["op"] == "Identity"
            and not deps[nid]
            and node.get("metrics", {}).get("weight_tensors", 0) > 0
        )

    for nid in topo:
        if is_aux_node(nid) and succ[nid]:
            stage_raw[nid] = max(0, min(stage_raw[child] for child in succ[nid]) - 1)

    used_stages = sorted(set(stage_raw.values()))
    stage_remap = {old_stage: new_stage for new_stage, old_stage in enumerate(used_stages)}
    col = {nid: stage_remap[stage_raw[nid]] for nid in stage_raw}

    max_col = max(col.values(), default=0)
    columns = [[] for _ in range(max_col + 1)]
    for node in nodes:
        columns[col[node["id"]]].append(node["id"])

    op_rank_map = {
        "Sub": 0,
        "Div": 0,
        "Clip": 0,
        "Concat": 1,
        "Conv": 2,
        "Gemm": 2,
        "MatMul": 2,
        "Relu": 3,
        "Sigmoid": 3,
        "Tanh": 3,
        "Mul": 4,
        "Add": 4,
        "Exp": 5,
        "RandomNormalLike": 6,
        "Identity": 7,
        "Constant": 8,
    }

    def lane_key(nid):
        branches = sorted(node_branches.get(nid, []))
        if not branches:
            return 3, 999, 0
        if len(branches) > 1:
            return 0, -len(branches), branches[0]
        if is_aux_node(nid):
            return 2, branches[0], 0
        return 1, branches[0], 0

    def op_rank(nid):
        return op_rank_map.get(node_lookup[nid]["op"], 5)

    def row_positions(cols):
        mapping = {}
        for column in cols:
            for row_index, nid in enumerate(column):
                mapping[nid] = row_index
        return mapping

    def barycenter(neighbors, positions, fallback):
        rows = [positions[n] for n in neighbors if n in positions]
        if not rows:
            return fallback
        return sum(rows) / len(rows)

    for ci in range(len(columns)):
        columns[ci].sort(key=lambda nid: (
            lane_key(nid),
            depth.get(nid, 0),
            op_rank(nid),
            -node_lookup[nid].get("metrics", {}).get("fanout", 0),
            nid,
        ))

    for _ in range(8):
        rp = row_positions(columns)
        for ci in range(1, len(columns)):
            columns[ci].sort(key=lambda nid: (
                lane_key(nid),
                barycenter(deps[nid], rp, rp.get(nid, 0)),
                op_rank(nid),
                -node_lookup[nid].get("metrics", {}).get("fanout", 0),
                nid,
            ))

        rp = row_positions(columns)
        for ci in range(len(columns) - 2, -1, -1):
            columns[ci].sort(key=lambda nid: (
                lane_key(nid),
                barycenter(succ[nid], rp, rp.get(nid, 0)),
                op_rank(nid),
                node_lookup[nid].get("metrics", {}).get("fanin", 0),
                nid,
            ))

    stage_meta = []
    for stage_index, stage_nodes in enumerate(columns):
        ops = {}
        branch_union = set()
        shared_count = 0
        aux_count = 0
        depths = []

        for nid in stage_nodes:
            op = node_lookup[nid]["op"]
            ops[op] = ops.get(op, 0) + 1
            branch_union.update(node_branches.get(nid, set()))
            if len(node_branches.get(nid, set())) > 1:
                shared_count += 1
            if is_aux_node(nid):
                aux_count += 1
            depths.append(depth.get(nid, stage_index))

        top_ops = [
            op for op, _ in sorted(ops.items(), key=lambda item: (-item[1], item[0]))[:3]
        ]

        stage_meta.append({
            "stage": stage_index,
            "count": len(stage_nodes),
            "top_ops": top_ops,
            "depth_min": min(depths) if depths else stage_index,
            "depth_max": max(depths) if depths else stage_index,
            "branches": len(branch_union),
            "shared": shared_count,
            "aux": aux_count,
        })

    node_branch_lists = {nid: sorted(list(branches)) for nid, branches in node_branches.items()}
    return columns, col, stage_meta, node_branch_lists


def _render_tensor_preview(tensors, limit=10):
    if not tensors:
        return '<div class="io-empty">none</div>'

    lines = []
    for item in tensors[:limit]:
        name = html_lib.escape(str(item.get("name", "?")))
        dtype = html_lib.escape(str(item.get("dtype", "?")))
        shape = html_lib.escape(_shape_text(item.get("shape", ["?"])))
        lines.append(
            f'<div class="io-line"><span class="io-name">{name}</span>'
            f'<span class="io-meta">{dtype} [{shape}]</span></div>'
        )

    if len(tensors) > limit:
        lines.append(f'<div class="io-more">+{len(tensors) - limit} more</div>')

    return "".join(lines)


HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ONNX Viz - {filename}</title>
<style>
:root {{
  --gold: #f0e68c;
  --red:  #c0392b;
  --bg:   #0a0a0a;
  --fg:   #ddd;
  --fg2:  #888;
}}
* {{ margin:0; padding:0; box-sizing:border-box; }}
body {{
  background: var(--bg);
  color: var(--fg);
  font-family: 'Courier New', monospace;
  min-height: 100vh;
}}
header {{
  padding: 18px 28px 12px;
  border-bottom: 1px solid #1a1a1a;
}}
header h1 {{
  font-size: 11px;
  letter-spacing: 3px;
  color: var(--gold);
  text-transform: uppercase;
  font-weight: normal;
}}
header p {{
  font-size: 9px;
  color: #555;
  margin-top: 4px;
  letter-spacing: 1px;
}}
#stats {{
  display: flex;
  gap: 18px;
  padding: 10px 28px;
  border-bottom: 1px solid #111;
  flex-wrap: wrap;
}}
.stat {{ font-size: 9px; color: #555; letter-spacing: 1px; }}
.stat span {{ color: var(--gold); }}

#graph-meta {{
  display: flex;
  gap: 18px;
  padding: 8px 28px 10px;
  border-bottom: 1px solid #111;
  flex-wrap: wrap;
}}
.meta-item {{ font-size: 8px; color: #555; letter-spacing: 1px; }}
.meta-item span {{ color: #b7b17a; }}

#io-panels {{
  display: grid;
  grid-template-columns: repeat(2, minmax(260px, 1fr));
  gap: 12px;
  padding: 12px 28px;
  border-bottom: 1px solid #111;
}}
.io-col {{
  border: 1px solid #171717;
  background: #0b0b0b;
  padding: 10px 12px;
  min-height: 72px;
}}
.io-col h2 {{
  font-size: 8px;
  color: #666;
  letter-spacing: 1px;
  text-transform: uppercase;
  margin-bottom: 7px;
  font-weight: normal;
}}
.io-line {{
  display: flex;
  justify-content: space-between;
  gap: 10px;
  font-size: 8px;
  color: #888;
  line-height: 1.5;
}}
.io-name {{ color: #b8b8b8; max-width: 60%; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }}
.io-meta {{ color: #666; white-space: nowrap; }}
.io-more, .io-empty {{
  margin-top: 4px;
  font-size: 8px;
  color: #555;
}}

#canvas-wrap {{
  position: relative;
  overflow: auto;
}}
canvas {{
  display: block;
  cursor: crosshair;
}}

/* Tooltip */
#tooltip {{
  position: fixed;
  display: none;
  pointer-events: none;
  background: #0c0c0c;
  border: 1px solid #1e1e1e;
  padding: 0;
  min-width: 280px;
  max-width: 390px;
  z-index: 100;
}}
#tooltip::before {{
  content: '';
  position: absolute;
  top:0; left:0; width:12px; height:12px;
  border-top: 2px solid var(--gold);
  border-left: 2px solid var(--gold);
}}
#tooltip::after {{
  content: '';
  position: absolute;
  bottom:0; right:0; width:12px; height:12px;
  border-bottom: 2px solid #333;
  border-right: 2px solid #333;
}}
#tt-inner {{
  padding: 14px 16px;
}}
#tt-op {{
  font-size: 10px;
  letter-spacing: 2px;
  color: var(--gold);
  margin-bottom: 8px;
  text-transform: uppercase;
}}
#tt-body {{
  font-size: 9px;
  color: #888;
  line-height: 1.75;
}}
#tt-body strong {{ color: #bbb; font-weight: normal; }}
#tt-hist {{
  margin-top: 8px;
  display: flex;
  align-items: flex-end;
  gap: 1px;
  height: 28px;
}}
.hist-bar {{
  flex: 1;
  background: var(--gold);
  opacity: 0.5;
  min-width: 3px;
}}

/* legend */
#legend {{
  display: flex;
  gap: 20px;
  padding: 8px 28px;
  border-top: 1px solid #111;
  flex-wrap: wrap;
}}
.leg {{ font-size: 8px; color: #555; display: flex; align-items: center; gap: 6px; letter-spacing: 1px; }}
.leg-dot {{ width: 10px; height: 10px; border-radius: 50%; }}

@media (max-width: 920px) {{
  #io-panels {{
    grid-template-columns: 1fr;
  }}
}}
</style>
</head>
<body>

<header>
  <h1>{filename}</h1>
  <p>{producer} &nbsp;·&nbsp; graph {graph_name} &nbsp;·&nbsp; ONNX opset {opset} &nbsp;·&nbsp; IR v{ir_version}</p>
</header>

<div id="stats">
  <div class="stat">PARAMS <span>{total_params}</span></div>
  {op_stats}
  <div class="stat">INPUTS <span>{n_inputs}</span></div>
  <div class="stat">OUTPUTS <span>{n_outputs}</span></div>
</div>

<div id="graph-meta">
  <div class="meta-item">NODES <span>{n_nodes}</span></div>
  <div class="meta-item">EDGES <span>{n_edges}</span></div>
  <div class="meta-item">DEPTH <span>{graph_depth}</span></div>
  <div class="meta-item">STAGES <span>{n_stages}</span></div>
  <div class="meta-item">TENSORS <span>{n_tensors}</span></div>
  <div class="meta-item">INITIALIZERS <span>{n_initializers}</span></div>
  <div class="meta-item">MAX FANIN <span>{max_fanin}</span></div>
  <div class="meta-item">MAX FANOUT <span>{max_fanout}</span></div>
</div>

<div id="io-panels">
  <div class="io-col">
    <h2>network inputs</h2>
    {inputs_preview}
  </div>
  <div class="io-col">
    <h2>network outputs</h2>
    {outputs_preview}
  </div>
</div>

<div id="canvas-wrap">
  <canvas id="c"></canvas>
</div>

<div id="legend">
  <div class="leg"><div class="leg-dot" style="background:#f0e68c;opacity:.9"></div>data node</div>
  <div class="leg"><div class="leg-dot" style="background:#f0e68c;opacity:.35;border:1px solid #f0e68c"></div>weight / const</div>
  <div class="leg"><div class="leg-dot" style="background:#c0392b;opacity:.9"></div>output node</div>
  <div class="leg"><div class="leg-dot" style="background:#3a3a00;border:1px solid #f0e68c"></div>input node</div>
</div>

<div id="tooltip">
  <div id="tt-inner">
    <div id="tt-op"></div>
    <div id="tt-body"></div>
    <div id="tt-hist"></div>
  </div>
</div>

<script>
const MODEL = {model_json};
const columns = {columns_json};
const nodeCol = {node_col_json};
const stageMeta = {stage_meta_json};
const edges = MODEL.edges || [];

const inputNames = new Set(MODEL.inputs.map(function(i) {{ return i.name; }}));
const outputNames = new Set(MODEL.outputs.map(function(o) {{ return o.name; }}));

const nodeById = {{}};
MODEL.nodes.forEach(function(n) {{ nodeById[n.id] = n; }});

const NODE_R = 9;
const COL_GAP = 138;
const ROW_GAP = 46;
const PAD_X = 70;
const PAD_Y = 72;
const MAX_SHOW = Math.max(14, Math.min(30, Math.floor((window.innerHeight - 280) / ROW_GAP)));

function splitColumn(col) {{
  if (col.length <= MAX_SHOW) {{
    return {{ shown: col.slice(), hidden: 0, head: col.length, tail: 0 }};
  }}
  const head = Math.max(8, Math.floor(MAX_SHOW * 0.65));
  const tail = MAX_SHOW - head;
  return {{
    shown: col.slice(0, head).concat(col.slice(col.length - tail)),
    hidden: col.length - MAX_SHOW,
    head: head,
    tail: tail,
  }};
}}

const nodePos = {{}};
const colMeta = [];

columns.forEach(function(col, ci) {{
  const split = splitColumn(col);
  const shownSet = new Set(split.shown);

  split.shown.forEach(function(nid, localRi) {{
    const isTail = split.hidden > 0 && localRi >= split.head;
    const row = isTail ? localRi + 1 : localRi;
    nodePos[nid] = {{
      x: PAD_X + ci * COL_GAP,
      y: PAD_Y + row * ROW_GAP,
      col: ci,
      row: row,
      shown: true,
    }};
  }});

  col.forEach(function(nid) {{
    if (!shownSet.has(nid)) {{
      nodePos[nid] = {{
        x: PAD_X + ci * COL_GAP,
        y: -9999,
        col: ci,
        row: -1,
        shown: false,
      }};
    }}
  }});

  colMeta.push({{
    count: col.length,
    hidden: split.hidden,
    shownCount: split.shown.length + (split.hidden > 0 ? 1 : 0),
    breakRow: split.hidden > 0 ? split.head : -1,
  }});
}});

const numCols = Math.max(columns.length, 1);
const maxRows = Math.max.apply(null, colMeta.map(function(c) {{ return c.shownCount; }}).concat([1]));
const W = PAD_X * 2 + Math.max(0, numCols - 1) * COL_GAP;
const H = PAD_Y * 2 + Math.max(0, maxRows - 1) * ROW_GAP;

const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
const DPR = window.devicePixelRatio || 1;

function resize() {{
  canvas.style.width = W + 'px';
  canvas.style.height = H + 'px';
  canvas.width = Math.floor(W * DPR);
  canvas.height = Math.floor(H * DPR);
  ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  draw();
}}

function nodeColor(n) {{
  const isOut = n.outputs.some(function(o) {{ return outputNames.has(o); }});
  if (isOut) return {{ fill:'#1a0000', stroke:'#c0392b', dot:'#c0392b' }};

  const isIn = n.inputs.some(function(i) {{ return inputNames.has(i); }});
  if (isIn) return {{ fill:'#1a1400', stroke:'#f0e68c', dot:'#f0e68c' }};

  if (n.op === 'Constant' || n.op === 'Identity') {{
    return {{ fill:'#0d0d0d', stroke:'#f0e68c', dot:'#555', opacity:0.45 }};
  }}
  return {{ fill:'#0d0d0d', stroke:'#f0e68c', dot:'#f0e68c' }};
}}

function shapeText(shape) {{
  if (!shape || !shape.length) return '?';
  return shape.join('x');
}}

function shortName(name) {{
  if (!name) return '(anonymous)';
  const parts = String(name).split('/');
  return parts[parts.length - 1] || name;
}}

function attrValue(v) {{
  if (v === null || v === undefined) return '';
  if (typeof v === 'object') {{
    const raw = JSON.stringify(v);
    if (raw.length > 80) return raw.slice(0, 77) + '...';
    return raw;
  }}
  const raw = String(v);
  if (raw.length > 80) return raw.slice(0, 77) + '...';
  return raw;
}}

function escapeHtml(value) {{
  return String(value ?? '')
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;')
    .replaceAll("'", '&#39;');
}}

function tensorBrief(t) {{
  return escapeHtml(shortName(t.name)) +
    ' <span style="color:#666">' +
    escapeHtml(String(t.dtype || '?')) +
    ' [' + escapeHtml(shapeText(t.shape || [])) + ']</span>';
}}

function draw(hovered) {{
  ctx.clearRect(0, 0, W, H);

  columns.forEach(function(col, ci) {{
    const x = PAD_X + ci * COL_GAP;
    const meta = colMeta[ci];
    const sm = stageMeta[ci] || {{}};
    const topOps = Array.isArray(sm.top_ops) ? sm.top_ops : [];
    const label = topOps.slice(0, 2).join('+') || 'flow';
    const depthLabel = (sm.depth_min !== undefined)
      ? ('d' + sm.depth_min + (sm.depth_max !== sm.depth_min ? '-' + sm.depth_max : ''))
      : '';

    const bandLeft = ci === 0 ? PAD_X - COL_GAP * 0.45 : PAD_X + (ci - 0.5) * COL_GAP;
    const bandW = COL_GAP * 0.9;
    ctx.fillStyle = ci % 2 === 0 ? 'rgba(255,255,255,0.012)' : 'rgba(240,230,140,0.015)';
    ctx.fillRect(bandLeft, PAD_Y - 36, bandW, H - PAD_Y + 28);

    ctx.font = '7px Courier New';
    ctx.fillStyle = '#444';
    ctx.textAlign = 'center';
    ctx.fillText(label, x, PAD_Y - 30);
    ctx.fillStyle = '#2a2a2a';
    ctx.fillText('stage ' + ci, x, PAD_Y - 20);
    if (depthLabel) {{
      ctx.fillStyle = '#3a3a3a';
      ctx.fillText(depthLabel, x, PAD_Y - 10);
    }}

    if (meta.hidden > 0) {{
      const y = PAD_Y + meta.breakRow * ROW_GAP + 12;
      ctx.fillStyle = '#353535';
      ctx.fillText('... ' + meta.hidden + ' hidden ...', x, y);

      ctx.strokeStyle = 'rgba(255,255,255,0.06)';
      ctx.lineWidth = 0.6;
      ctx.beginPath();
      ctx.moveTo(x - NODE_R - 2, y - 5);
      ctx.lineTo(x + NODE_R + 2, y - 5);
      ctx.stroke();
    }}
  }});

  ctx.lineCap = 'round';
  edges.forEach(function(edge) {{
    const from = nodePos[edge.from];
    const to = nodePos[edge.to];
    if (!from || !to || !from.shown || !to.shown) return;

    const span = Math.max(1, to.col - from.col);
    const alpha = Math.min(0.38, 0.11 + span * 0.03);
    const lineW = Math.min(1.8, 0.5 + Math.log2((edge.count || 1) + 1) * 0.35);

    const toNode = nodeById[edge.to];
    const isOutputPath = toNode && toNode.outputs.some(function(o) {{ return outputNames.has(o); }});

    const deltaX = to.x - from.x;
    const bend = Math.max(20, Math.min(74, deltaX * 0.35));
    const c1x = from.x + bend;
    const c2x = to.x - bend;

    ctx.beginPath();
    ctx.moveTo(from.x + NODE_R, from.y);
    ctx.bezierCurveTo(c1x, from.y, c2x, to.y, to.x - NODE_R, to.y);
    ctx.strokeStyle = isOutputPath
      ? 'rgba(192,57,43,' + alpha + ')'
      : 'rgba(240,230,140,' + (alpha * 0.8) + ')';
    ctx.lineWidth = lineW;
    ctx.stroke();
  }});

  MODEL.nodes.forEach(function(n) {{
    const pos = nodePos[n.id];
    if (!pos || !pos.shown) return;

    const c = nodeColor(n);
    const isHovered = hovered && hovered.id === n.id;

    ctx.globalAlpha = c.opacity || 1;

    if (isHovered) {{
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, NODE_R + 5, 0, Math.PI * 2);
      ctx.strokeStyle = c.stroke;
      ctx.lineWidth = 0.5;
      ctx.globalAlpha = 0.3;
      ctx.stroke();
      ctx.globalAlpha = 1;
    }}

    ctx.beginPath();
    ctx.arc(pos.x, pos.y, NODE_R, 0, Math.PI * 2);
    ctx.fillStyle = isHovered ? '#1a1a00' : c.fill;
    ctx.fill();
    ctx.strokeStyle = c.stroke;
    ctx.lineWidth = isHovered ? 2 : 1.2;
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(pos.x, pos.y, isHovered ? 3.5 : 2.5, 0, Math.PI * 2);
    ctx.fillStyle = c.dot;
    ctx.fill();

    ctx.globalAlpha = 1;

    ctx.font = '7px Courier New';
    ctx.fillStyle = isHovered ? '#f0e68c' : '#333';
    ctx.textAlign = 'center';
    ctx.fillText((n.op || '').slice(0, 10), pos.x, pos.y + NODE_R + 9);
  }});
}}

const tooltip = document.getElementById('tooltip');
const ttOp = document.getElementById('tt-op');
const ttBody = document.getElementById('tt-body');
const ttHist = document.getElementById('tt-hist');

function showTooltip(n, px, py) {{
  const m = n.metrics || {{}};
  ttOp.textContent = n.op || '(unknown)';

  let html = '';
  html += '<strong>name</strong> ' + escapeHtml(shortName(n.name || n.op)) + '<br>';
  html += '<strong>id</strong> ' + n.id +
    '  <strong>stage</strong> ' + (m.stage_hint ?? nodeCol[n.id] ?? 0) +
    '  <strong>depth</strong> ' + (m.depth_hint ?? nodeCol[n.id] ?? 0) + '<br>';
  html += '<strong>fanin</strong> ' + (m.fanin ?? 0) + '  <strong>fanout</strong> ' + (m.fanout ?? 0) + '<br>';
  html += '<strong>inputs</strong> ' + (m.input_count ?? n.inputs.length) +
    '  <strong>outputs</strong> ' + (m.output_count ?? n.outputs.length) + '<br>';

  if ((m.branch_count || 0) > 0) {{
    html += '<strong>flow lanes</strong> ' + m.branch_count;
    if (Array.isArray(m.branch_ids) && m.branch_ids.length) {{
      html += ' (' + m.branch_ids.join(',') + ')';
    }}
    html += '<br>';
  }}

  if ((m.weight_tensors || 0) > 0) {{
    html += '<strong>weights</strong> ' + m.weight_tensors +
      ' tensor(s), ' + Number(m.weight_params || 0).toLocaleString() + ' params<br>';
  }}

  const attrPairs = Object.entries(n.attrs || {{}});
  if (attrPairs.length) {{
    const attrPreview = attrPairs.slice(0, 6).map(function(entry) {{
      return escapeHtml(entry[0]) + '=' + escapeHtml(attrValue(entry[1]));
    }}).join('<br>');
    html += '<br><strong>attrs</strong><br>' + attrPreview;
    if (attrPairs.length > 6) {{
      html += '<br><span style="color:#666">+' + (attrPairs.length - 6) + ' more attrs</span>';
    }}
  }}

  const inPreview = (n.io && n.io.inputs ? n.io.inputs : []).slice(0, 4);
  if (inPreview.length) {{
    html += '<br><br><strong>input tensors</strong><br>' + inPreview.map(tensorBrief).join('<br>');
  }}

  const outPreview = (n.io && n.io.outputs ? n.io.outputs : []).slice(0, 3);
  if (outPreview.length) {{
    html += '<br><strong>output tensors</strong><br>' + outPreview.map(tensorBrief).join('<br>');
  }}

  const weightNames = n.weight_names || [];
  if (weightNames.length && MODEL.weights) {{
    const firstName = weightNames[0];
    const w = MODEL.weights[firstName];
    if (w) {{
      html += '<br><strong>weight sample</strong> ' + escapeHtml(shortName(firstName)) +
        ' [' + escapeHtml(shapeText(w.shape || [])) + '] ' + escapeHtml(String(w.dtype || '?')) + '<br>';

      if (typeof w.mean === 'number' && typeof w.std === 'number') {{
        html += '<strong>mean</strong> ' + w.mean + '  <strong>std</strong> ' + w.std + '<br>';
      }}
      if (typeof w.min === 'number' && typeof w.max === 'number') {{
        html += '<strong>range</strong> [' + w.min + ', ' + w.max + ']<br>';
      }}
      if (typeof w.sparsity === 'number') {{
        html += '<strong>near-zero</strong> ' + (w.sparsity * 100).toFixed(1) + '%';
      }}

      if (Array.isArray(w.hist) && w.hist.length) {{
        const maxH = Math.max.apply(null, w.hist.concat([1]));
        ttHist.innerHTML = w.hist.map(function(v) {{
          const h = Math.round((v / maxH) * 28);
          return '<div class="hist-bar" style="height:' + h + 'px"></div>';
        }}).join('');
      }} else {{
        ttHist.innerHTML = '';
      }}
    }} else {{
      ttHist.innerHTML = '';
    }}
  }} else {{
    ttHist.innerHTML = '';
  }}

  const isIn = n.inputs.some(function(i) {{ return inputNames.has(i); }});
  const isOut = n.outputs.some(function(o) {{ return outputNames.has(o); }});
  if (isIn) html += '<br><strong style="color:#f0e68c">left boundary (network input)</strong>';
  if (isOut) html += '<br><strong style="color:#c0392b">right boundary (network output)</strong>';

  ttBody.innerHTML = html;

  tooltip.style.display = 'block';
  const tw = tooltip.offsetWidth;
  const th = tooltip.offsetHeight;

  let lx = px + 18;
  let ly = py - 10;
  if (lx + tw > window.innerWidth - 10) lx = px - tw - 10;
  if (ly + th > window.innerHeight - 10) ly = py - th - 10;
  tooltip.style.left = lx + 'px';
  tooltip.style.top = ly + 'px';
}}

function hideTooltip() {{
  tooltip.style.display = 'none';
}}

function hitNode(mx, my) {{
  for (const n of MODEL.nodes) {{
    const pos = nodePos[n.id];
    if (!pos || !pos.shown) continue;
    const dx = mx - pos.x;
    const dy = my - pos.y;
    if (dx * dx + dy * dy <= (NODE_R + 4) * (NODE_R + 4)) return n;
  }}
  return null;
}}

canvas.addEventListener('mousemove', function(e) {{
  const rect = canvas.getBoundingClientRect();
  const mx = e.clientX - rect.left;
  const my = e.clientY - rect.top;
  const n = hitNode(mx, my);
  if (n) {{
    canvas.style.cursor = 'pointer';
    draw(n);
    showTooltip(n, e.clientX, e.clientY);
  }} else {{
    canvas.style.cursor = 'crosshair';
    draw(null);
    hideTooltip();
  }}
}});

canvas.addEventListener('mouseleave', function() {{
  draw(null);
  hideTooltip();
}});

resize();
window.addEventListener('resize', resize);
</script>
</body>
</html>
"""


def generate_html(model_data, out_path):
    meta = model_data["meta"]
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
        node["metrics"]["branch_ids"] = branch_ids[:6]

    sorted_ops = sorted(meta["op_counts"].items(), key=lambda item: (-item[1], item[0]))
    top_ops = sorted_ops[:10]
    op_stats = " ".join(
        f'<div class="stat">{html_lib.escape(op)} <span>{cnt}</span></div>'
        for op, cnt in top_ops
    )
    if len(sorted_ops) > len(top_ops):
        op_stats += f' <div class="stat">OTHER OPS <span>{len(sorted_ops) - len(top_ops)}</span></div>'

    graph_depth = max(node_col.values(), default=-1) + 1

    html = HTML_TEMPLATE.format(
        filename=html_lib.escape(meta["filename"]),
        graph_name=html_lib.escape(meta.get("graph_name", "unnamed_graph")),
        producer=html_lib.escape(meta["producer"] or "unknown"),
        opset=", ".join(str(v) for v in meta["opset"]) if meta["opset"] else "unknown",
        ir_version=meta["ir_version"],
        total_params=f"{meta['total_params']:,}",
        op_stats=op_stats,
        n_inputs=len(inputs),
        n_outputs=len(outputs),
        n_nodes=meta.get("node_count", len(nodes)),
        n_edges=meta.get("edge_count", len(model_data.get("edges", []))),
        graph_depth=graph_depth,
        n_stages=len(columns),
        n_tensors=meta.get("tensor_count", len(model_data.get("tensors", []))),
        n_initializers=meta.get("initializer_count", len(model_data.get("weights", {}))),
        max_fanin=meta.get("max_fanin", 0),
        max_fanout=meta.get("max_fanout", 0),
        inputs_preview=_render_tensor_preview(inputs, limit=10),
        outputs_preview=_render_tensor_preview(outputs, limit=10),
        model_json=json.dumps(model_data, separators=(",", ":")),
        columns_json=json.dumps(columns, separators=(",", ":")),
        node_col_json=json.dumps(node_col, separators=(",", ":")),
        stage_meta_json=json.dumps(stage_meta, separators=(",", ":")),
    )

    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)

    print(f"Written: {out_path}")
    print(f"  Nodes:  {len(nodes)}")
    print(f"  Edges:  {meta.get('edge_count', 0)}")
    print(f"  Params: {meta['total_params']:,}")
    print(f"  Cols:   {len(columns)}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python vizzer.py model.onnx [output.html]")
        sys.exit(1)

    onnx_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else onnx_path.replace(".onnx", "_viz.html")

    print(f"Loading: {onnx_path}")
    data = extract_model(onnx_path)
    generate_html(data, out_path)
