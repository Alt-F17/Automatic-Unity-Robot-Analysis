import argparse
import html
import importlib
import inspect
import json
import os
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from string import Template
from urllib.parse import urlparse

import numpy as np
import onnx
from onnx import AttributeProto, TensorProto, numpy_helper


BASE_DIR = os.path.dirname(__file__)
HTML_TEMPLATE_PATH = os.path.join(BASE_DIR, "netronizer-template.html")
CSS_TEMPLATE_PATH = os.path.join(BASE_DIR, "netronizer-template.css")
JS_TEMPLATE_PATH = os.path.join(BASE_DIR, "netronizer-template.js")


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
                weight_info.update(
                    {
                        "mean": round(float(np.mean(arr)), 5),
                        "std": round(float(np.std(arr)), 5),
                        "min": round(float(np.min(arr)), 5),
                        "max": round(float(np.max(arr)), 5),
                        "sparsity": round(float(np.mean(np.abs(arr) < 0.01)), 4),
                        "hist": [int(x) for x in hist],
                        "hist_edges": [round(float(x), 4) for x in edges],
                    }
                )

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

        nodes.append(
            {
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
            }
        )

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
                for name in node["inputs"]
                if name
            ][:8],
            "outputs": [
                {
                    "name": name,
                    "shape": tensors.get(name, {}).get("shape", ["?"]),
                    "dtype": tensors.get(name, {}).get("dtype", "?"),
                }
                for name in node["outputs"]
                if name
            ][:8],
        }

    tensor_list = []
    for name, spec in tensors.items():
        tensor_list.append(
            {
                "name": name,
                "shape": spec.get("shape", ["?"]),
                "dtype": spec.get("dtype", "?"),
                "roles": sorted(spec.get("roles", [])),
                "consumers": len(tensor_consumers.get(name, [])),
            }
        )
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
    - stage assignment by topological depth (with optional compression)
    - aux/constant nodes pinned just before their first consumer
    - output-ancestry lanes to keep branches visually grouped
    - barycentric row ordering (20 passes) + local swap crossing minimization
    - aux nodes kept in a separate bottom zone so they don't displace the main flow
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
        return sum(rows) / len(rows) if rows else fallback

    def split_aux(column):
        reg = [nid for nid in column if not is_aux_node(nid)]
        aux = [nid for nid in column if is_aux_node(nid)]
        return reg, aux

    for ci in range(len(columns)):
        reg, aux = split_aux(columns[ci])
        reg.sort(
            key=lambda nid: (
                lane_key(nid),
                depth.get(nid, 0),
                op_rank(nid),
                -node_lookup[nid].get("metrics", {}).get("fanout", 0),
                nid,
            )
        )
        aux.sort(
            key=lambda nid: (
                min((depth.get(c, 0) for c in succ[nid]), default=0),
                nid,
            )
        )
        columns[ci] = reg + aux

    for iteration in range(20):
        rp = row_positions(columns)

        if iteration % 2 == 0:
            for ci in range(1, len(columns)):
                reg, aux = split_aux(columns[ci])
                n_reg = len(reg)
                reg.sort(
                    key=lambda nid: (
                        lane_key(nid),
                        barycenter(
                            [d for d in deps[nid] if not is_aux_node(d)],
                            rp,
                            rp.get(nid, 0),
                        ),
                        op_rank(nid),
                        -node_lookup[nid].get("metrics", {}).get("fanout", 0),
                        nid,
                    )
                )
                aux.sort(key=lambda nid: (barycenter(succ[nid], rp, n_reg), nid))
                columns[ci] = reg + aux
        else:
            for ci in range(len(columns) - 2, -1, -1):
                reg, aux = split_aux(columns[ci])
                n_reg = len(reg)
                reg.sort(
                    key=lambda nid: (
                        lane_key(nid),
                        barycenter(
                            [s for s in succ[nid] if not is_aux_node(s)],
                            rp,
                            rp.get(nid, 0),
                        ),
                        op_rank(nid),
                        node_lookup[nid].get("metrics", {}).get("fanin", 0),
                        nid,
                    )
                )
                aux.sort(key=lambda nid: (barycenter(succ[nid], rp, n_reg), nid))
                columns[ci] = reg + aux

    def crossing_count(col_a, col_b):
        row_a = {nid: i for i, nid in enumerate(col_a)}
        row_b = {nid: i for i, nid in enumerate(col_b)}
        pairs = [(row_a[n], row_b[c]) for n in col_a for c in succ[n] if c in row_b]
        count = 0
        for i in range(len(pairs)):
            for j in range(i + 1, len(pairs)):
                if (pairs[i][0] - pairs[j][0]) * (pairs[i][1] - pairs[j][1]) < 0:
                    count += 1
        return count

    def col_crossings(ci):
        total = 0
        if ci > 0:
            total += crossing_count(columns[ci - 1], columns[ci])
        if ci < len(columns) - 1:
            total += crossing_count(columns[ci], columns[ci + 1])
        return total

    for _ in range(4):
        improved = False
        for ci in range(len(columns)):
            for i in range(len(columns[ci]) - 1):
                before = col_crossings(ci)
                columns[ci][i], columns[ci][i + 1] = columns[ci][i + 1], columns[ci][i]
                after = col_crossings(ci)
                if after < before:
                    improved = True
                else:
                    columns[ci][i], columns[ci][i + 1] = columns[ci][i + 1], columns[ci][i]
        if not improved:
            break

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

        top_ops = [op for op, _ in sorted(ops.items(), key=lambda item: (-item[1], item[0]))[:3]]

        stage_meta.append(
            {
                "stage": stage_index,
                "count": len(stage_nodes),
                "top_ops": top_ops,
                "depth_min": min(depths) if depths else stage_index,
                "depth_max": max(depths) if depths else stage_index,
                "branches": len(branch_union),
                "shared": shared_count,
                "aux": aux_count,
            }
        )

    node_branch_lists = {nid: sorted(list(branches)) for nid, branches in node_branches.items()}
    return columns, col, stage_meta, node_branch_lists


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


def _load_text_asset(path, label):
    try:
        with open(path, "r", encoding="utf-8") as asset_file:
            return asset_file.read()
    except FileNotFoundError as exc:
        raise RuntimeError(f"Missing {label}: {path}") from exc


def _build_frontend_html(app_title, dashboard):
    template = Template(_load_text_asset(HTML_TEMPLATE_PATH, "HTML template"))
    css_text = _load_text_asset(CSS_TEMPLATE_PATH, "CSS template")
    js_text = _load_text_asset(JS_TEMPLATE_PATH, "JS template")

    document = template.safe_substitute(
        PAGE_TITLE=html.escape(f"{app_title} - {dashboard['model_name']}"),
        APP_TITLE=html.escape(app_title),
        DASHBOARD_JSON=json.dumps(dashboard, separators=(",", ":")),
    )

    document = document.replace("/* __APP_CSS__ */", css_text)
    document = document.replace("/* __APP_JS__ */", js_text)
    return document


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
    try:
        document = _build_frontend_html(args.title, dashboard)
    except RuntimeError as exc:
        print(f"Error: failed to build frontend: {exc}")
        return 1

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
