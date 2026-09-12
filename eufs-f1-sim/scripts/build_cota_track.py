#!/usr/bin/env python3
"""Build COTA as an EUFS cone track from the published F1 Grand Prix plan view.

Does not use TUMFTM/GPS. Digitizes the COTA 20-turn layout, scales the closed
loop to the FIA 5.513 km lap, and places the four big_orange gate cones on the
orange start/finish line (where sector 1 begins on the F1 course map).
"""

from __future__ import annotations

import argparse
import hashlib
import math
import re
import shutil
from datetime import date
from pathlib import Path

FIA_LAP_M = 5513.0
FRONT_EXTENT_M = 4.40
GATE_CLEARANCE_M = 1.00
CONE_SPACING_M = 8.0
CONE_SPACING_MIN_M = 5.0
KAPPA_DENSE = 0.025
GATE_ALONG_M = 0.40
HALF_WIDTH_M = 7.0
BOUNDARY_STRIP_WIDTH_M = 0.20
BOUNDARY_STRIP_HEIGHT_M = 0.008
BOUNDARY_STRIP_Z_M = 0.0
BOUNDARY_STRIP_MESH = "cota_boundary_strips.dae"
COVARIANCE = (0.01, 0.01, 0.0)
SELECTOR_NAME = "cota"
TRACK_ID = "cota_layout_reference_v1"
FRAME = "cota_local_enu"
# Orange sector-1 path start on the 2022 F1 COTA course layout (SVG y-down).
ORANGE_SF_SVG = (55.3, 233.1)


def _yaml_scalar(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, int):
        return str(value)
    return '"' + str(value).replace("\\", "\\\\").replace('"', '\\"') + '"'


def _write_yaml(path: Path, data):
    lines = []

    def emit(obj, level):
        pad = "  " * level
        if isinstance(obj, dict):
            for key, value in obj.items():
                if isinstance(value, (dict, list)):
                    lines.append(f"{pad}{key}:")
                    emit(value, level + 1)
                else:
                    lines.append(f"{pad}{key}: {_yaml_scalar(value)}")
        elif isinstance(obj, list):
            for item in obj:
                if isinstance(item, dict):
                    first = True
                    for key, value in item.items():
                        prefix = "- " if first else "  "
                        first = False
                        if isinstance(value, (dict, list)):
                            lines.append(f"{pad}{prefix}{key}:")
                            emit(value, level + 2)
                        else:
                            lines.append(f"{pad}{prefix}{key}: {_yaml_scalar(value)}")
                else:
                    lines.append(f"{pad}- {_yaml_scalar(item)}")

    emit(data, 0)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_table(path: Path, header, rows):
    body = [",".join(header)]
    for row in rows:
        cells = []
        for item in row:
            if isinstance(item, float):
                cells.append(f"{item:.6f}")
            else:
                cells.append(str(item))
        body.append(",".join(cells))
    path.write_text("\n".join(body) + "\n", encoding="utf-8")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def _cubic(p0, p1, p2, p3, steps=20):
    pts = []
    for i in range(1, steps + 1):
        t = i / steps
        u = 1.0 - t
        x = u**3 * p0[0] + 3 * u**2 * t * p1[0] + 3 * u * t**2 * p2[0] + t**3 * p3[0]
        y = u**3 * p0[1] + 3 * u**2 * t * p1[1] + 3 * u * t**2 * p2[1] + t**3 * p3[1]
        pts.append((x, y))
    return pts


def _flatten_svg_path(d_attr: str):
    token_re = re.compile(r"([MmCcSsLlHhVvZz])|([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)")
    tokens = []
    for match in token_re.finditer(re.sub(r"\s+", " ", d_attr)):
        if match.group(1):
            tokens.append(("cmd", match.group(1)))
        else:
            tokens.append(("num", float(match.group(2))))

    def chunks(values, size):
        out = []
        while values:
            out.append(values[:size])
            del values[:size]
        return out

    pts = []
    i = 0
    cx = cy = sx = sy = 0.0
    prev = ""
    reflect = (0.0, 0.0)
    while i < len(tokens):
        kind, val = tokens[i]
        if kind != "cmd":
            raise SystemExit(f"unexpected SVG token {val}")
        cmd = val
        i += 1
        if cmd in "Zz":
            if pts and (abs(pts[-1][0] - sx) > 1e-9 or abs(pts[-1][1] - sy) > 1e-9):
                pts.append((sx, sy))
            continue
        nums = []
        while i < len(tokens) and tokens[i][0] == "num":
            nums.append(tokens[i][1])
            i += 1
        if cmd == "M":
            pairs = chunks(nums, 2)
            cx, cy = pairs[0]
            sx, sy = cx, cy
            pts.append((cx, cy))
            for x, y in pairs[1:]:
                cx, cy = x, y
                pts.append((cx, cy))
            prev = "L"
        elif cmd == "m":
            pairs = chunks(nums, 2)
            cx += pairs[0][0]
            cy += pairs[0][1]
            sx, sy = cx, cy
            pts.append((cx, cy))
            for dx, dy in pairs[1:]:
                cx += dx
                cy += dy
                pts.append((cx, cy))
            prev = "l"
        elif cmd in "Cc":
            rel = cmd == "c"
            for x1, y1, x2, y2, x, y in chunks(nums, 6):
                p0 = (cx, cy)
                p1 = (cx + x1, cy + y1) if rel else (x1, y1)
                p2 = (cx + x2, cy + y2) if rel else (x2, y2)
                p3 = (cx + x, cy + y) if rel else (x, y)
                pts.extend(_cubic(p0, p1, p2, p3))
                cx, cy = p3
                reflect = p2
            prev = cmd
        elif cmd in "Ss":
            rel = cmd == "s"
            for x2, y2, x, y in chunks(nums, 4):
                p0 = (cx, cy)
                p1 = (2 * cx - reflect[0], 2 * cy - reflect[1]) if prev in "CcSs" else p0
                p2 = (cx + x2, cy + y2) if rel else (x2, y2)
                p3 = (cx + x, cy + y) if rel else (x, y)
                pts.extend(_cubic(p0, p1, p2, p3))
                cx, cy = p3
                reflect = p2
            prev = cmd
        else:
            raise SystemExit(f"unhandled SVG command {cmd}")
    cleaned = [pts[0]]
    for point in pts[1:]:
        if math.hypot(point[0] - cleaned[-1][0], point[1] - cleaned[-1][1]) > 1e-6:
            cleaned.append(point)
    if math.hypot(cleaned[0][0] - cleaned[-1][0], cleaned[0][1] - cleaned[-1][1]) < 1e-3:
        cleaned = cleaned[:-1]
    return cleaned


def _extract_track_path(svg_text: str) -> str:
    match = re.search(r'<path class="st0" d="([^"]+)"', svg_text, re.S)
    if not match:
        raise SystemExit("COTA SVG is missing the Grand Prix outline path")
    return match.group(1)


def _extract_turn_svg(svg_text: str):
    block = re.search(r'<g id="コーナー番号">(.*?)</g>\s*<g id="DRS">', svg_text, re.S)
    if not block:
        return []
    unique = []
    for x, y in re.findall(r'cx="([0-9.]+)" cy="([0-9.]+)"', block.group(1)):
        point = (float(x), float(y))
        if not any(math.hypot(point[0] - prior[0], point[1] - prior[1]) <= 1e-3 for prior in unique):
            unique.append(point)
    return unique


def _polyline_s(xy):
    s_vals = [0.0]
    for i in range(len(xy) - 1):
        s_vals.append(s_vals[-1] + math.hypot(xy[i + 1][0] - xy[i][0], xy[i + 1][1] - xy[i][1]))
    return s_vals


def _nearest_index(xy, target):
    return min(range(len(xy)), key=lambda i: (xy[i][0] - target[0]) ** 2 + (xy[i][1] - target[1]) ** 2)


def _roll(seq, start):
    return seq[start:] + seq[:start]


def _geometry(center_xy):
    closed = center_xy + [center_xy[0]]
    s_closed = _polyline_s(closed)
    lap_m = s_closed[-1]
    headings = []
    kappa = []
    for i, point in enumerate(center_xy):
        nxt = center_xy[(i + 1) % len(center_xy)]
        prev = center_xy[(i - 1) % len(center_xy)]
        headings.append(math.atan2(nxt[1] - point[1], nxt[0] - point[0]))
        a1 = math.atan2(point[1] - prev[1], point[0] - prev[0])
        a2 = headings[-1]
        dang = (a2 - a1 + math.pi) % (2.0 * math.pi) - math.pi
        ds = math.hypot(nxt[0] - point[0], nxt[1] - point[1])
        kappa.append(0.0 if ds < 1e-6 else dang / ds)

    def sample(s_m):
        s_mod = s_m % lap_m
        idx = 0
        while idx < len(s_closed) - 1 and s_closed[idx + 1] < s_mod:
            idx += 1
        idx = min(idx, len(center_xy) - 1)
        nxt = (idx + 1) % len(center_xy)
        span = s_closed[idx + 1] - s_closed[idx]
        t = 0.0 if span <= 1e-12 else (s_mod - s_closed[idx]) / span
        x = center_xy[idx][0] + t * (center_xy[nxt][0] - center_xy[idx][0])
        y = center_xy[idx][1] + t * (center_xy[nxt][1] - center_xy[idx][1])
        yaw = headings[idx] + t * (((headings[nxt] - headings[idx] + math.pi) % (2.0 * math.pi) - math.pi))
        fwd = (math.cos(yaw), math.sin(yaw))
        left = (-fwd[1], fwd[0])
        return {
            "s": s_mod,
            "x": x,
            "y": y,
            "yaw": yaw,
            "fwd": fwd,
            "left": left,
            "kappa": kappa[idx],
            "left_xy": (x + left[0] * HALF_WIDTH_M, y + left[1] * HALF_WIDTH_M),
            "right_xy": (x - left[0] * HALF_WIDTH_M, y - left[1] * HALF_WIDTH_M),
        }

    return {"xy": center_xy, "lap_m": lap_m, "s_closed": s_closed, "headings": headings, "kappa": kappa, "sample": sample}


def _sample_boundary(geom, side):
    cones = []
    s_m = 0.0
    lap = geom["lap_m"]
    while s_m < lap - 0.5 * CONE_SPACING_MIN_M:
        sample = geom["sample"](s_m)
        xy = sample["left_xy"] if side == "left" else sample["right_xy"]
        cones.append((xy[0], xy[1]))
        s_m += CONE_SPACING_MIN_M if abs(sample["kappa"]) >= KAPPA_DENSE else CONE_SPACING_M
    return cones


def _build_rows(geom):
    left = _sample_boundary(geom, "left")
    right = _sample_boundary(geom, "right")
    gate = geom["sample"](0.0)
    plus = geom["sample"](GATE_ALONG_M)
    minus = geom["sample"](geom["lap_m"] - GATE_ALONG_M)
    big = [minus["left_xy"], plus["left_xy"], minus["right_xy"], plus["right_xy"]]
    spawn_s = (geom["lap_m"] - (FRONT_EXTENT_M + GATE_CLEARANCE_M)) % geom["lap_m"]
    spawn_xy = (
        gate["x"] - gate["fwd"][0] * (FRONT_EXTENT_M + GATE_CLEARANCE_M),
        gate["y"] - gate["fwd"][1] * (FRONT_EXTENT_M + GATE_CLEARANCE_M),
    )
    ahead = geom["sample"](spawn_s + 2.0)
    ds = ahead["s"] - spawn_s
    if ds < -0.5 * geom["lap_m"]:
        ds += geom["lap_m"]
    if ds <= 0.0:
        raise SystemExit("spawn forward projection does not increase s")
    rows = [("tag", "x", "y", "direction", "x_variance", "y_variance", "xy_covariance")]
    for x, y in left:
        rows.append(("blue", x, y, 0.0, *COVARIANCE))
    for x, y in right:
        rows.append(("yellow", x, y, 0.0, *COVARIANCE))
    for x, y in big:
        rows.append(("big_orange", x, y, 0.0, *COVARIANCE))
    rows.append(("car_start", spawn_xy[0], spawn_xy[1], gate["yaw"], 0.0, 0.0, 0.0))
    return {
        "left": left,
        "right": right,
        "big": big,
        "gate": gate,
        "spawn_xy": spawn_xy,
        "spawn_yaw": gate["yaw"],
        "spawn_s": spawn_s,
        "csv_rows": rows,
        "fwd": gate["fwd"],
    }


def _write_sdf(path: Path, built):
    chunks = ["<?xml version='1.0'?>\n<sdf version='1.6'>\n  <model name='track'>\n"]
    counts = {"blue_cone": 0, "yellow_cone": 0, "big_cone": 0}
    for tag, x, y, *_rest in built["csv_rows"][1:]:
        if tag == "blue":
            uri, name = "blue_cone", f"blue_cone_{counts['blue_cone']}"
            counts["blue_cone"] += 1
        elif tag == "yellow":
            uri, name = "yellow_cone", f"yellow_cone_{counts['yellow_cone']}"
            counts["yellow_cone"] += 1
        elif tag == "big_orange":
            uri, name = "big_cone", f"big_cone_{counts['big_cone']}"
            counts["big_cone"] += 1
        else:
            continue
        chunks.append(
            "    <include>\n"
            f"      <pose>{x:.6f} {y:.6f} 0.15 0 0 0</pose>\n"
            f"      <uri>model://{uri}</uri>\n"
            f"      <name>{name}</name>\n"
            '      <covariance x="0.01" y="0.01" xy="0.0"/>\n'
            "    </include>\n"
        )
    # Keep the cone parent model include-only.  Gazebo Classic can drop
    # direct include visuals when a second link is added to that model, so the
    # visual-only boundary mesh is included as a separate world model below.
    chunks.append("    <static>1</static>\n  </model>\n</sdf>\n")
    path.write_text("".join(chunks), encoding="utf-8")
    return counts


def _miter_ring(points):
    """Return left/right offset vertices for a closed, ordered polyline.

    The cone samples are ordered by increasing centerline s.  Computing the
    join from the incoming and outgoing directions keeps the ribbon width
    constant through corners while avoiding the visible gaps produced by
    independently placed boxes.
    """
    half = 0.5 * BOUNDARY_STRIP_WIDTH_M
    ring = []
    for i, point in enumerate(points):
        previous = points[(i - 1) % len(points)]
        following = points[(i + 1) % len(points)]
        incoming = (point[0] - previous[0], point[1] - previous[1])
        outgoing = (following[0] - point[0], following[1] - point[1])
        incoming_norm = math.hypot(*incoming)
        outgoing_norm = math.hypot(*outgoing)
        if incoming_norm < 1e-9 or outgoing_norm < 1e-9:
            raise SystemExit("cannot build boundary strip with coincident cone samples")
        incoming = (incoming[0] / incoming_norm, incoming[1] / incoming_norm)
        outgoing = (outgoing[0] / outgoing_norm, outgoing[1] / outgoing_norm)
        incoming_normal = (-incoming[1], incoming[0])
        outgoing_normal = (-outgoing[1], outgoing[0])
        miter = (
            incoming_normal[0] + outgoing_normal[0],
            incoming_normal[1] + outgoing_normal[1],
        )
        miter_norm = math.hypot(*miter)
        if miter_norm < 1e-9:
            miter = outgoing_normal
            miter_norm = 1.0
        else:
            miter = (miter[0] / miter_norm, miter[1] / miter_norm)
        denominator = miter[0] * outgoing_normal[0] + miter[1] * outgoing_normal[1]
        if denominator <= 1e-6:
            miter = outgoing_normal
            scale = half
        else:
            # The cap prevents a near-reversal in a sampled polyline from
            # producing a long spike while retaining the miter join.
            scale = min(half / denominator, 4.0 * half)
        offset = (miter[0] * scale, miter[1] * scale)
        ring.append(
            (
                (point[0] + offset[0], point[1] + offset[1]),
                (point[0] - offset[0], point[1] - offset[1]),
            )
        )
    return ring


def _write_boundary_mesh(path: Path, built):
    """Write both closed boundary ribbons as one compact Collada visual.

    Each side is an independent closed solid.  The only loop closure is the
    last-to-first join on that same side, so the finish gate never gets a
    cross-track white segment.
    """
    vertices = []
    triangles = []
    for points in (built["left"], built["right"]):
        ring = _miter_ring(points)
        base = len(vertices)
        for left, right in ring:
            vertices.extend(
                (
                    (left[0], left[1], BOUNDARY_STRIP_Z_M),
                    (right[0], right[1], BOUNDARY_STRIP_Z_M),
                    (left[0], left[1], BOUNDARY_STRIP_Z_M + BOUNDARY_STRIP_HEIGHT_M),
                    (right[0], right[1], BOUNDARY_STRIP_Z_M + BOUNDARY_STRIP_HEIGHT_M),
                )
            )
        count = len(ring)
        for i in range(count):
            j = (i + 1) % count
            il, ir, ibl, ibr = (base + 4 * i + k for k in (0, 1, 2, 3))
            jl, jr, jbl, jbr = (base + 4 * j + k for k in (0, 1, 2, 3))
            # top, bottom, and both vertical sides: a watertight quad strip.
            for quad in (
                (ibl, ibr, jbr, jbl),
                (il, ir, jr, jl),
                (il, jl, jbl, ibl),
                (ir, ibr, jbr, jr),
            ):
                a, b, c, d = quad
                triangles.extend((a, b, c, a, c, d))

    position_values = " ".join(
        f"{x:.9f} {y:.9f} {z:.9f}" for x, y, z in vertices
    )
    index_values = " ".join(str(index) for index in triangles)
    path.write_text(
        "<?xml version='1.0' encoding='utf-8'?>\n"
        "<COLLADA xmlns='http://www.collada.org/2005/11/COLLADASchema' version='1.4.1'>\n"
        "  <asset><unit name='meter' meter='1'/><up_axis>Z_UP</up_axis></asset>\n"
        "  <library_effects>\n"
        "    <effect id='boundary_white_effect'><profile_COMMON><technique sid='common'>"
        "<phong><diffuse><color>1 1 1 1</color></diffuse>"
        "<specular><color>0.1 0.1 0.1 1</color></specular></phong>"
        "</technique></profile_COMMON></effect>\n"
        "  </library_effects>\n"
        "  <library_materials><material id='boundary_white_material' name='White'>"
        "<instance_effect url='#boundary_white_effect'/></material></library_materials>\n"
        "  <library_geometries>\n"
        "    <geometry id='cota_boundary_strips_geometry' name='COTA white boundary strips'>\n"
        "      <mesh>\n"
        f"        <source id='cota_boundary_strips_positions'><float_array id='cota_boundary_strips_positions_array' count='{len(vertices) * 3}'>{position_values}</float_array>"
        f"<technique_common><accessor source='#cota_boundary_strips_positions_array' count='{len(vertices)}' stride='3'>"
        "<param name='X' type='float'/><param name='Y' type='float'/><param name='Z' type='float'/>"
        "</accessor></technique_common></source>\n"
        "        <vertices id='cota_boundary_strips_vertices'><input semantic='POSITION' source='#cota_boundary_strips_positions'/></vertices>\n"
        f"        <triangles material='boundary_white_material' count='{len(triangles) // 3}'><input semantic='VERTEX' source='#cota_boundary_strips_vertices' offset='0'/><p>{index_values}</p></triangles>\n"
        "      </mesh>\n"
        "    </geometry>\n"
        "  </library_geometries>\n"
        "  <library_visual_scenes><visual_scene id='cota_boundary_scene' name='COTA boundary strips'><node id='cota_boundary_strips'>"
        "<instance_geometry url='#cota_boundary_strips_geometry'><bind_material><technique_common>"
        "<instance_material symbol='boundary_white_material' target='#boundary_white_material'/>"
        "</technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>\n"
        "  <scene><instance_visual_scene url='#cota_boundary_scene'/></scene>\n"
        "</COLLADA>\n",
        encoding="utf-8",
    )


def _write_world(path: Path, built, geom):
    xs = [p[0] for p in geom["xy"]]
    ys = [p[1] for p in geom["xy"]]
    cx = 0.5 * (min(xs) + max(xs))
    cy = 0.5 * (min(ys) + max(ys))
    spawn_x, spawn_y = built["spawn_xy"]
    back_x = spawn_x - 28.0 * built["fwd"][0]
    back_y = spawn_y - 28.0 * built["fwd"][1]
    path.write_text(
        f"""<sdf version='1.6'>
  <world name='cota'>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>{back_x:.3f} {back_y:.3f} 12.0 0 0.40 {built['spawn_yaw']:.5f}</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
    <scene>
      <ambient>0.40 0.40 0.43 1.0</ambient>
      <shadows>false</shadows>
    </scene>
    <spherical_coordinates>
      <heading_deg>180</heading_deg>
      <latitude_deg>30.13278</latitude_deg>
      <longitude_deg>-97.64111</longitude_deg>
    </spherical_coordinates>
    <light name='sun' type='directional'>
      <cast_shadows>0</cast_shadows>
      <pose frame=''>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>4000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.5 -1</direction>
    </light>
    <model name='cota_ground'>
      <static>1</static>
      <pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>4000 4000</size>
            </plane>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>4000 4000</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.22 0.26 0.20 1</ambient>
            <diffuse>0.28 0.32 0.24 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name='cota_boundary_strips'>
      <static>1</static>
      <pose>0 0 0.01 0 0 0</pose>
      <link name='boundary_strip_link'>
        <visual name='boundary_strips'>
          <geometry>
            <mesh>
              <uri>model://cota/{BOUNDARY_STRIP_MESH}</uri>
            </mesh>
          </geometry>
          <material>
            <lighting>false</lighting>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <emissive>1 1 1 1</emissive>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
    <include>
      <uri>model://cota</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>
  </world>
</sdf>
""",
        encoding="utf-8",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--svg", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()

    svg_text = args.svg.read_text(encoding="utf-8")
    svg_xy = _flatten_svg_path(_extract_track_path(svg_text))
    length_svg = sum(
        math.hypot(svg_xy[i][0] - svg_xy[i - 1][0], svg_xy[i][1] - svg_xy[i - 1][1])
        for i in range(1, len(svg_xy))
    ) + math.hypot(svg_xy[0][0] - svg_xy[-1][0], svg_xy[0][1] - svg_xy[-1][1])
    scale = FIA_LAP_M / length_svg
    # SVG y-down -> local y-up, then metres.
    metric = [(x * scale, -y * scale) for x, y in svg_xy]
    orange_metric = (ORANGE_SF_SVG[0] * scale, -ORANGE_SF_SVG[1] * scale)
    gate_i = _nearest_index(metric, orange_metric)
    metric = _roll(metric, gate_i)
    # Translate so the orange start/finish is the origin.
    origin = metric[0]
    metric = [(x - origin[0], y - origin[1]) for x, y in metric]
    geom = _geometry(metric)
    fit_error = abs(geom["lap_m"] - FIA_LAP_M)
    if fit_error > 25.0:
        raise SystemExit(f"scaled lap {geom['lap_m']:.3f} m is too far from FIA {FIA_LAP_M} m")
    built = _build_rows(geom)

    out = args.out
    csv_dir = out / "csv"
    model_dir = out / "models" / "cota"
    world_dir = out / "worlds"
    source_dir = out / "source"
    meta_dir = out / "cota"
    for directory in (csv_dir, model_dir, world_dir, source_dir, meta_dir):
        directory.mkdir(parents=True, exist_ok=True)

    source_copy = source_dir / "2022_F1_CourseLayout_COTA.svg"
    if args.svg.resolve() != source_copy.resolve():
        shutil.copy2(args.svg, source_copy)
    (source_dir / "LICENSE.txt").write_text(
        "Source map: Wikimedia File:2022 F1 CourseLayout COTA.svg\n"
        "License: Creative Commons Attribution-ShareAlike 4.0 International\n"
        "https://creativecommons.org/licenses/by-sa/4.0/\n"
        "Author: ごひょううべこ (14 October 2022)\n"
        "This derived COTA EUFS cone layout is a metric approximation of that plan view.\n",
        encoding="utf-8",
    )

    csv_path = csv_dir / "cota.csv"
    sdf_path = model_dir / "model.sdf"
    mesh_path = model_dir / BOUNDARY_STRIP_MESH
    world_path = world_dir / "cota.world"
    _write_table(csv_path, built["csv_rows"][0], built["csv_rows"][1:])
    _write_boundary_mesh(mesh_path, built)
    counts = _write_sdf(sdf_path, built)
    _write_world(world_path, built, geom)
    (model_dir / "model.config").write_text(
        """<?xml version="1.0" ?>
<model>
    <name>cota</name>
    <version>1.0</version>
    <sdf version="1.6">model.sdf</sdf>
    <author>
        <name>EUFS F1 Sim</name>
        <email>eufs-f1-sim@local</email>
    </author>
    <description>COTA Grand Prix layout_reference cone track. Gate is the orange start/finish line.</description>
</model>
""",
        encoding="utf-8",
    )

    turn_svg = _extract_turn_svg(svg_text)
    events = [
        {
            "id": "start_finish",
            "type": "gate",
            "s_m": 0.0,
            "x_m": built["gate"]["x"],
            "y_m": built["gate"]["y"],
            "heading_rad": built["gate"]["yaw"],
            "notes": "Four big_orange cones on the orange F1 start/finish line (sector 1 start).",
        }
    ]
    projected_turns = []
    for sx, sy in turn_svg:
        pt = (sx * scale - origin[0], -sy * scale - origin[1])
        idx = _nearest_index(geom["xy"], pt)
        sample = geom["sample"](geom["s_closed"][idx])
        projected_turns.append((sample["s"], sample))
    projected_turns.sort(key=lambda item: item[0])
    if any(left[0] >= right[0] for left, right in zip(projected_turns, projected_turns[1:])):
        raise SystemExit("COTA turn landmarks are not strictly ordered after projection")
    # The SVG contains 40 circles (two painted circles per label), and its
    # source order is not lap order: it lists 1–9, 12–20, 10, 11.  Deduplicate
    # first, project all labels, sort by increasing map progress, then assign
    # the canonical turn number.  This keeps event IDs tied to the repaired
    # centerline order and avoids stale paired labels being numbered twice.
    for turn_i, (_, sample) in enumerate(projected_turns, start=1):
        events.append(
            {
                "id": f"turn_{turn_i}",
                "type": "corner",
                "s_m": round(sample["s"], 3),
                "x_m": sample["x"],
                "y_m": sample["y"],
                "heading_rad": sample["yaw"],
                "notes": "Turn number from the COTA F1 course layout, projected onto the scaled centerline.",
            }
        )
    events.append(
        {
            "id": "t1_uphill_approach",
            "type": "corner_sector",
            "s_m": round(events[1]["s_m"] * 0.85, 3) if len(events) > 1 else 0.0,
            "x_m": geom["sample"](events[1]["s_m"] * 0.85)["x"] if len(events) > 1 else 0.0,
            "y_m": geom["sample"](events[1]["s_m"] * 0.85)["y"] if len(events) > 1 else 0.0,
            "heading_rad": geom["sample"](events[1]["s_m"] * 0.85)["yaw"] if len(events) > 1 else 0.0,
            "notes": "FIA uphill Turn 1 approach; elevation is not modelled (z unknown/zero).",
        }
    )

    hashes = {
        "csv": _sha256(csv_path),
        "model_sdf": _sha256(sdf_path),
        "world": _sha256(world_path),
        "boundary_mesh": _sha256(mesh_path),
    }
    xs = [p[0] for p in geom["xy"]]
    ys = [p[1] for p in geom["xy"]]
    provenance = {
        "track_id": TRACK_ID,
        "selector_name": SELECTOR_NAME,
        "fidelity": "layout_reference",
        "frame": FRAME,
        "origin": "Orange start/finish line (F1 sector-1 start) after y-up flip and FIA scale",
        "yaw_convention": "counter-clockwise from +x",
        "elevation": "unknown/zero — 2D COTA plan view does not reproduce gradients",
        "lap_length_m": round(geom["lap_m"], 6),
        "fia_reference_lap_length_m": FIA_LAP_M,
        "fit_error_m": round(fit_error, 6),
        "turn_count_source": 20,
        "direction": "counter-clockwise COTA Grand Prix layout",
        "bbox_m": {"x_min": min(xs), "x_max": max(xs), "y_min": min(ys), "y_max": max(ys)},
        "cone_policy": {
            "blue": "left boundary along travel direction",
            "yellow": "right boundary along travel direction",
            "big_orange": "four-cone gate measured on the orange start/finish line",
            "spacing_m": CONE_SPACING_M,
            "spacing_min_m": CONE_SPACING_MIN_M,
            "half_width_m": HALF_WIDTH_M,
            "width_note": "authored constant 14 m envelope from the plan view, not surveyed boundaries",
            "collision_mode": "cone_cylinder",
            "white_strip_width_m": BOUNDARY_STRIP_WIDTH_M,
            "white_strip_height_m": BOUNDARY_STRIP_HEIGHT_M,
            "white_strip_mesh": f"models/cota/{BOUNDARY_STRIP_MESH}",
            "white_strip_join": "miter_joined_quad_strip; each side closes at finish seam independently",
            "white_strip_gate_policy": "no cross-track connection across the four-cone orange gate",
            "white_strip_marker_namespace": "eufs_track_boundary",
            "white_strip_source": "ordered_blue_yellow_cones",
        },
        "spawn": {
            "front_extent_m": FRONT_EXTENT_M,
            "gate_clearance_m": GATE_CLEARANCE_M,
            "gate_x_m": built["gate"]["x"],
            "gate_y_m": built["gate"]["y"],
            "gate_heading_rad": built["gate"]["yaw"],
            "spawn_s_m": built["spawn_s"],
            "spawn_x_m": built["spawn_xy"][0],
            "spawn_y_m": built["spawn_xy"][1],
            "spawn_yaw_rad": built["spawn_yaw"],
            "direction_check": "increasing_s",
        },
        # Launch attachment metadata is explicit so the generic selector can
        # preserve the established N=1 pose without guessing cone ordering.
        "launch_spawn": {
            "spawn_x_m": -3.904850706197952,
            "spawn_y_m": 3.1228419364078146,
            "spawn_yaw_rad": -0.6745807971885812,
        },
        "grid": {"spawn_arclength_back_m": 5.0},
        "counts": {
            "blue": counts["blue_cone"],
            "yellow": counts["yellow_cone"],
            "big_orange": counts["big_cone"],
            "car_start": 1,
        },
        "hashes": hashes,
        "source_urls": [
            "https://commons.wikimedia.org/wiki/File:2022_F1_CourseLayout_COTA.svg",
            "https://www.fia.com/sites/default/files/2017_usa_preview_0.pdf",
            "https://www.fia.com/sites/default/files/final_formula1crypto.commiamigrandprix2024_lo_0.pdf",
        ],
        "license_note": "Plan-view SVG is CC BY-SA 4.0. Derived EUFS csv/sdf/world are a layout_reference of Circuit of The Americas, not TUMFTM GPS.",
        "retrieved_at": date.today().isoformat(),
        "generator": "eufs-f1-sim/scripts/build_cota_track.py",
        "scale_m_per_svg_unit": scale,
        "limitations": [
            "Digitized from the published COTA F1 plan view; not a surveyed boundary.",
            "TUMFTM Austin.csv was not used.",
            "z_m is unknown/zero.",
            "Results are integration geometry, not COTA lap-time claims.",
        ],
    }
    _write_yaml(model_dir / "provenance.yaml", provenance)
    shutil.copy2(model_dir / "provenance.yaml", meta_dir / "provenance.yaml")
    _write_yaml(meta_dir / "events.yaml", {"track_id": TRACK_ID, "events": events})
    _write_table(
        meta_dir / "centerline.csv",
        ["s_m", "x_m", "y_m", "z_m", "yaw_rad", "curvature_1pm"],
        [
            (geom["s_closed"][i], p[0], p[1], 0.0, geom["headings"][i], geom["kappa"][i])
            for i, p in enumerate(geom["xy"])
        ],
    )
    _write_table(
        meta_dir / "boundaries.csv",
        ["s_m", "left_x_m", "left_y_m", "right_x_m", "right_y_m"],
        [
            (
                geom["sample"](geom["s_closed"][i])["s"],
                geom["sample"](geom["s_closed"][i])["left_xy"][0],
                geom["sample"](geom["s_closed"][i])["left_xy"][1],
                geom["sample"](geom["s_closed"][i])["right_xy"][0],
                geom["sample"](geom["s_closed"][i])["right_xy"][1],
            )
            for i in range(len(geom["xy"]))
        ],
    )
    for name in ("events.yaml", "centerline.csv", "boundaries.csv"):
        shutil.copy2(meta_dir / name, model_dir / name)
    print(
        "COTA layout_reference "
        f"lap={geom['lap_m']:.3f}m fit_error={fit_error:.3f}m "
        f"blue={counts['blue_cone']} yellow={counts['yellow_cone']} "
        f"gate={counts['big_cone']} spawn={built['spawn_xy']}"
    )


if __name__ == "__main__":
    main()
