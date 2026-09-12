#!/usr/bin/env python3
"""
Export GrabCAD Mercedes F1 STEP assembly to per-link STL meshes for eufs_racecar.

Uses Open CASCADE (cadquery-ocp) for STEP import and STL tessellation.
Solid grouping is derived from assembly bounding boxes (see classify_solids()).
Wheel meshes are centered on their link origins; joint origins are written to
step_export.json and optionally patched into racecar.xacro.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

TARGET_WHEELBASE_M = 3.28
MESH_DEFLECTION_MM = 0.8
TINY_VOLUME_MM3 = 50_000.0
# Rim shares the tire center (~3 mm); leftover body uprights sit ~140 mm inboard.
WHEEL_COMPANION_MM = 50.0

LINK_NAMES = (
    "chassis",
    "front_wing",
    "rear_wing",
    "left_front_wheel",
    "right_front_wheel",
    "left_rear_wheel",
    "right_rear_wheel",
)
WHEEL_LINKS = LINK_NAMES[3:]


@dataclass
class SolidInfo:
    index: int
    center_mm: tuple[float, float, float]
    extent_mm: tuple[float, float, float]
    volume_mm3: float


def read_solids(step_path: Path) -> list:
    reader = STEPControl_Reader()
    if reader.ReadFile(str(step_path)) != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed: {step_path}")
    reader.TransferRoots()
    shape = reader.OneShape()

    solids = []
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        solids.append(exp.Current())
        exp.Next()
    return solids


def solid_bounds_mm(solid) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    bb = Bnd_Box()
    BRepBndLib.Add_s(solid, bb)
    xmin, ymin, zmin, xmax, ymax, zmax = bb.Get()
    return (xmin, ymin, zmin), (xmax, ymax, zmax)


def solid_info(solid, index: int) -> SolidInfo:
    lo, hi = solid_bounds_mm(solid)
    center = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)
    extent = (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(solid, props)
    return SolidInfo(index=index, center_mm=center, extent_mm=extent, volume_mm3=props.Mass())


def _corner_link(info: SolidInfo, mid_z: float) -> str:
    """CAD +X is ROS right; low CAD Z is the nose."""
    is_front = info.center_mm[2] < mid_z
    is_right = info.center_mm[0] > 0
    if is_front:
        return "right_front_wheel" if is_right else "left_front_wheel"
    return "right_rear_wheel" if is_right else "left_rear_wheel"


def _center_dist_mm(a: SolidInfo, b: SolidInfo) -> float:
    dx = a.center_mm[0] - b.center_mm[0]
    dy = a.center_mm[1] - b.center_mm[1]
    dz = a.center_mm[2] - b.center_mm[2]
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def classify_solids(solids: list) -> tuple[dict[str, set[int]], set[int]]:
    """Group STEP solids into chassis and four corner wheels (tire + rim only).

    GrabCAD assembly is one 'Formula 1' body plus four 'WHEEL' instances.
    Each WHEEL instance is a tire (largest solid at that corner) and a rim at
    the same center. The body also has leftover suspension solids at each
    corner (uprights / wishbones / brake ducts, ~140 mm inboard of the tire).
    Those leftovers are dropped so they do not poke out of the rubber.
    Front/rear wings are already tessellated inside the body solid, so the
    wing links stay empty (dummy STLs at export).
    """
    infos = [solid_info(s, i) for i, s in enumerate(solids)]
    by_volume = sorted(infos, key=lambda item: item.volume_mm3, reverse=True)

    chassis = {by_volume[0].index}
    candidates = [info for info in infos if info.index not in chassis]

    wheel_z = [info.center_mm[2] for info in candidates if info.volume_mm3 >= TINY_VOLUME_MM3]
    if len(wheel_z) < 4:
        raise RuntimeError("Could not find enough large solids to infer wheel locations")
    wheel_z.sort()
    front_z = sum(wheel_z[:2]) / 2
    rear_z = sum(wheel_z[-2:]) / 2
    mid_z = (front_z + rear_z) / 2

    groups: dict[str, set[int]] = {name: set() for name in LINK_NAMES}
    dropped: set[int] = set()
    by_corner: dict[str, list[SolidInfo]] = {name: [] for name in WHEEL_LINKS}

    for info in candidates:
        if info.volume_mm3 < TINY_VOLUME_MM3:
            chassis.add(info.index)
            continue
        by_corner[_corner_link(info, mid_z)].append(info)

    for name, members in by_corner.items():
        if not members:
            raise RuntimeError(f"No solids classified for {name}")
        tire = max(members, key=lambda item: item.volume_mm3)
        groups[name].add(tire.index)
        for info in members:
            if info.index == tire.index:
                continue
            if _center_dist_mm(info, tire) <= WHEEL_COMPANION_MM:
                groups[name].add(info.index)
            else:
                dropped.add(info.index)

    groups["chassis"] = chassis

    assigned = set().union(*groups.values()) | dropped
    if assigned != set(range(len(solids))):
        missing = set(range(len(solids))) - assigned
        groups["chassis"].update(missing)

    for name in WHEEL_LINKS:
        if not groups[name]:
            raise RuntimeError(f"No solids classified for {name}")

    return groups, dropped


def cad_mm_to_ros_m(x: float, y: float, z: float, scale: float) -> tuple[float, float, float]:
    """CAD (Y-up) → ROS (Z-up), metres.

    STEP frame: +X left, +Y up, +Z aft (low Z = front).
    ROS frame:  +X forward, +Y left, +Z up.

    This is a proper rotation (det > 0): π about ROS X applied to the old
    Y-down mapping that exported the car inverted (halo below, wheels on top).
      X_ros = -Z_cad
      Y_ros = -X_cad
      Z_ros =  Y_cad
    """
    return (-z * scale, -x * scale, y * scale)


def make_compound(solids: list) -> TopoDS_Compound:
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    for solid in solids:
        builder.Add(comp, solid)
    return comp


def corners_ros(solids: list, scale: float) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for solid in solids:
        lo, hi = solid_bounds_mm(solid)
        for x, y, z in (
            lo,
            (hi[0], lo[1], lo[2]),
            (lo[0], hi[1], lo[2]),
            (hi[0], hi[1], lo[2]),
            (lo[0], lo[1], hi[2]),
            (hi[0], lo[1], hi[2]),
            (lo[0], hi[1], hi[2]),
            hi,
        ):
            points.append(cad_mm_to_ros_m(x, y, z, scale))
    return points


def bounds_ros(solids: list, scale: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    points = corners_ros(solids, scale)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def group_center_ros(solids: list, indices: set[int], scale: float) -> tuple[float, float, float]:
    pts = [cad_mm_to_ros_m(*solid_info(solids[i], i).center_mm, scale) for i in indices]
    n = len(pts)
    return (sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n, sum(p[2] for p in pts) / n)


def z_extent(points: list[tuple[float, float, float]]) -> tuple[float, float]:
    zs = [p[2] for p in points]
    return min(zs), max(zs)


def wheelbase_mm(solids: list, link_groups: dict[str, set[int]]) -> float:
    front = link_groups["left_front_wheel"] | link_groups["right_front_wheel"]
    rear = link_groups["left_rear_wheel"] | link_groups["right_rear_wheel"]
    front_z = sum(solid_info(solids[i], i).center_mm[2] for i in front) / len(front)
    rear_z = sum(solid_info(solids[i], i).center_mm[2] for i in rear) / len(rear)
    return rear_z - front_z


def ros_transform(scale: float) -> gp_Trsf:
    """Same mapping as cad_mm_to_ros_m: CAD Y-up → ROS Z-up, det > 0."""
    tr = gp_Trsf()
    tr.SetValues(
        0, 0, -scale, 0,
        -scale, 0, 0, 0,
        0, scale, 0, 0,
    )
    return tr


def rewrite_binary_stl(out_path: Path) -> None:
    """RViz2 only loads binary STL; Open CASCADE often writes ASCII."""
    from stl import mesh as stlmesh
    from stl.stl import BINARY

    loaded = stlmesh.Mesh.from_file(str(out_path))
    loaded.save(str(out_path), mode=BINARY)


def export_stl(shape, out_path: Path, transform: gp_Trsf, deflection_mm: float) -> None:
    transformed = BRepBuilderAPI_Transform(shape, transform, True).Shape()
    mesh = BRepMesh_IncrementalMesh(transformed, deflection_mm, False, 0.5, True)
    mesh.Perform()
    if not mesh.IsDone():
        raise RuntimeError(f"meshing failed for {out_path.name}")
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(transformed, str(out_path)):
        raise RuntimeError(f"STL write failed: {out_path}")
    rewrite_binary_stl(out_path)


def mesh_points(shape, transform: gp_Trsf, deflection_mm: float) -> list[tuple[float, float, float]]:
    transformed = BRepBuilderAPI_Transform(shape, transform, True).Shape()
    mesher = BRepMesh_IncrementalMesh(transformed, deflection_mm, False, 0.5, True)
    mesher.Perform()
    points: list[tuple[float, float, float]] = []
    exp = TopExp_Explorer(transformed, TopAbs_FACE)
    while exp.More():
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(TopoDS.Face_s(exp.Current()), loc)
        if tri is not None:
            trsf = loc.Transformation()
            for i in range(1, tri.NbNodes() + 1):
                pnt = tri.Node(i)
                pnt.Transform(trsf)
                points.append((pnt.X(), pnt.Y(), pnt.Z()))
        exp.Next()
    if not points:
        raise RuntimeError("meshing produced no vertices")
    return points


def wheel_link_rotation() -> gp_Trsf:
    """Chassis-frame upright tire (disk in XZ, axle +Y) -> wheel link (disk in XY, axle +Z).

    Rear/front wheel joints use rpy 1.5708 0 0, which maps link Z onto the
    chassis lateral axis so the tire stands wheels-down after this pre-rotation.
    """
    tr = gp_Trsf()
    tr.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), -math.pi / 2)
    return tr


def stl_wheel_metrics(path: Path) -> tuple[float, float]:
    """Radius in the XY disk and width along Z (link frame after wheel_link_rotation)."""
    from stl import mesh as stlmesh
    import numpy as np

    verts = stlmesh.Mesh.from_file(str(path)).vectors.reshape(-1, 3)
    radius = float(np.sqrt(verts[:, 0] ** 2 + verts[:, 1] ** 2).max())
    width = float(verts[:, 2].max() - verts[:, 2].min())
    return radius, width


def apply_shift(point: tuple[float, float, float], shift: tuple[float, float, float]) -> tuple[float, float, float]:
    return (point[0] + shift[0], point[1] + shift[1], point[2] + shift[2])


def group_center_shifted(
    solids: list,
    indices: set[int],
    scale: float,
    shift: tuple[float, float, float],
) -> tuple[float, float, float]:
    pts = [
        apply_shift(cad_mm_to_ros_m(*solid_info(solids[i], i).center_mm, scale), shift)
        for i in indices
    ]
    n = len(pts)
    return (sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n, sum(p[2] for p in pts) / n)


def corners_shifted(
    solids: list,
    scale: float,
    shift: tuple[float, float, float],
) -> list[tuple[float, float, float]]:
    return [apply_shift(point, shift) for point in corners_ros(solids, scale)]


def patch_wheel_collision_macros(
    macros_path: Path, wheel_radius: float, wheel_width: float
) -> None:
    text = macros_path.read_text(encoding="utf-8")
    cyl_pattern = (
        r'(<xacro:macro name="(?:left|right)_wheels_collision_geometry">\s*'
        r'<origin xyz="0 0 0" rpy=")[^"]+(" />\s*'
        r'<geometry>\s*'
        r'<cylinder length=")[0-9.]+(" radius=")[0-9.]+(" />)'
    )
    cyl_repl = rf'\g<1>0 0 0\g<2>{wheel_width:.3f}\g<3>{wheel_radius:.3f}\g<4>'
    text, count = re.subn(cyl_pattern, cyl_repl, text, count=2)
    if count != 2:
        raise RuntimeError(f"Failed to patch wheel collision cylinders in {macros_path}")
    macros_path.write_text(text, encoding="utf-8")


def patch_racecar_xacro(
    xacro_path: Path,
    joints: dict[str, tuple[float, float, float]],
    sensor_z: float,
) -> None:
    text = xacro_path.read_text(encoding="utf-8")
    replacements = {
        "left_rear_wheel_joint": (joints["left_rear_wheel"], "1.5708 0 0"),
        "right_rear_wheel_joint": (joints["right_rear_wheel"], "1.5708 0 0"),
        "left_steering_hinge_joint": (joints["left_front_wheel"], "0 1.5708 0"),
        "right_steering_hinge_joint": (joints["right_front_wheel"], "0 1.5708 0"),
    }
    for joint_name, ((x, y, z), rpy) in replacements.items():
        pattern = rf'(<joint name="{re.escape(joint_name)}" type="[^"]+">\s*<origin xyz=")[^"]+(" rpy=")[^"]+(" />)'
        repl = rf'\g<1>{x:.4f} {y:.4f} {z:.4f}\g<2>{rpy}\g<3>'
        new_text, count = re.subn(pattern, repl, text, count=1)
        if count != 1:
            raise RuntimeError(f"Failed to patch joint {joint_name} in {xacro_path}")
        text = new_text

    laser_z = sensor_z
    camera_z = max(0.3, sensor_z - 0.10)
    sensor_patches = (
        (
            r'(<joint name="hokuyo_joint" type="fixed">\s*<origin xyz=")[^"]+(" rpy="0 0 0"/>)',
            rf'\g<1>0.0 0.0 {laser_z:.4f}\g<2>',
        ),
        (
            r'(<joint name="zed_camera_joint" type="fixed">\s*<origin xyz=")[^"]+(" rpy="0 0 0"/>)',
            rf'\g<1>0.35 0 {camera_z:.4f}\g<2>',
        ),
    )
    for pattern, repl in sensor_patches:
        new_text, count = re.subn(pattern, repl, text, count=1)
        if count != 1:
            raise RuntimeError(f"Failed to patch sensor joint in {xacro_path}: {pattern}")
        text = new_text

    xacro_path.write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--step",
        type=Path,
        default=Path("/home/gera/Downloads/Assem step.STEP"),
        help="GrabCAD STEP file",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("overlay/eufs_racecar/eufs_racecar/meshes"),
    )
    parser.add_argument("--wheelbase", type=float, default=TARGET_WHEELBASE_M)
    parser.add_argument(
        "--metadata",
        type=Path,
        default=Path("overlay/eufs_racecar/eufs_racecar/meshes/step_export.json"),
    )
    parser.add_argument(
        "--patch-xacro",
        type=Path,
        default=Path("overlay/eufs_racecar/eufs_racecar/urdf/racecar.xacro"),
        help="Patch wheel joint origins in this xacro after export",
    )
    parser.add_argument(
        "--patch-macros",
        type=Path,
        default=Path("overlay/eufs_racecar/eufs_racecar/urdf/macros.xacro"),
        help="Patch wheel collision cylinder sizes",
    )
    args = parser.parse_args()

    if not args.step.is_file():
        print(f"STEP not found: {args.step}", file=sys.stderr)
        return 1

    solids = read_solids(args.step)
    print(f"Read {len(solids)} solids from {args.step.name}")

    link_groups, dropped = classify_solids(solids)
    for name in LINK_NAMES:
        print(f"  {name}: {sorted(link_groups[name])}")
    if dropped:
        print(
            "  dropped leftover body solids (uprights/wishbones/brake ducts): "
            f"{sorted(dropped)}"
        )

    wb_mm = wheelbase_mm(solids, link_groups)
    scale = args.wheelbase / wb_mm
    print(f"Wheelbase CAD {wb_mm:.1f} mm -> scale {scale:.6f} m/mm (target {args.wheelbase} m)")

    tr_scale = ros_transform(scale)

    lr = group_center_ros(solids, link_groups["left_rear_wheel"], scale)
    rr = group_center_ros(solids, link_groups["right_rear_wheel"], scale)
    rear_mid = ((lr[0] + rr[0]) / 2, (lr[1] + rr[1]) / 2, (lr[2] + rr[2]) / 2)

    wheel_members = [solids[i] for name in WHEEL_LINKS for i in link_groups[name]]
    wheel_pts = mesh_points(make_compound(wheel_members), tr_scale, MESH_DEFLECTION_MM)
    ground_z = min(p[2] for p in wheel_pts)
    print(f"Wheel mesh ground z (pre-shift) {ground_z:.3f} m")

    shift_tuple = (-rear_mid[0], -rear_mid[1], -ground_z)
    shift = gp_Vec(*shift_tuple)
    tr_pre = gp_Trsf()
    tr_pre.SetTranslation(shift)
    tr = tr_pre.Multiplied(tr_scale)

    args.out_dir.mkdir(parents=True, exist_ok=True)

    joint_positions: dict[str, tuple[float, float, float]] = {}
    wheel_radii: list[float] = []
    wheel_widths: list[float] = []
    rx_wheel = wheel_link_rotation()

    dummy_box = BRepPrimAPI_MakeBox(0.001, 0.001, 0.001).Shape()

    for link in LINK_NAMES:
        members = [solids[i] for i in sorted(link_groups[link])]
        out = args.out_dir / f"{link}.STL"

        if not members:
            export_stl(dummy_box, out, gp_Trsf(), 0.5)
            print(f"  {link}: {out.name} dummy (geometry is in chassis)")
            continue

        compound = make_compound(members)

        if link in WHEEL_LINKS:
            center = group_center_shifted(solids, link_groups[link], scale, shift_tuple)
            joint_positions[link] = center
            local_shift = gp_Trsf()
            local_shift.SetTranslation(gp_Vec(-center[0], -center[1], -center[2]))
            export_tr = rx_wheel.Multiplied(local_shift.Multiplied(tr))
        else:
            export_tr = tr

        export_stl(compound, out, export_tr, MESH_DEFLECTION_MM)
        if link in WHEEL_LINKS:
            radius, width = stl_wheel_metrics(out)
            wheel_radii.append(radius)
            wheel_widths.append(width)
            print(
                f"  {link}: {out.name} ({out.stat().st_size // 1024} KiB) "
                f"r={radius:.3f} width={width:.3f} joint_z={center[2]:.3f}"
            )
        else:
            z_lo, z_hi = z_extent(corners_shifted(members, scale, shift_tuple))
            print(f"  {link}: {out.name} ({out.stat().st_size // 1024} KiB) z~[{z_lo:.3f},{z_hi:.3f}]")

    for name in ("left_steering_hinge", "right_steering_hinge"):
        export_stl(dummy_box, args.out_dir / f"{name}.STL", gp_Trsf(), 0.5)

    wheel_radius = sum(wheel_radii) / len(wheel_radii)
    wheel_width = sum(wheel_widths) / len(wheel_widths)
    for link, radius in zip(WHEEL_LINKS, wheel_radii):
        x, y, _ = joint_positions[link]
        joint_positions[link] = (x, y, radius)
    all_lo, all_hi = bounds_ros([solids[i] for i in range(len(solids))], scale)
    all_lo = apply_shift(all_lo, shift_tuple)
    all_hi = apply_shift(all_hi, shift_tuple)

    chassis_pts = mesh_points(
        make_compound([solids[i] for i in link_groups["chassis"]]), tr, MESH_DEFLECTION_MM
    )
    chassis_z = z_extent(chassis_pts)
    wheel_z_centers = [joint_positions[name][2] for name in WHEEL_LINKS]
    mean_wheel_z = sum(wheel_z_centers) / len(wheel_z_centers)
    left_y = joint_positions["left_rear_wheel"][1]
    right_y = joint_positions["right_rear_wheel"][1]
    if mean_wheel_z >= chassis_z[1]:
        raise RuntimeError(
            f"Car still inverted after Y-up map: wheels z={mean_wheel_z:.3f} "
            f">= chassis top {chassis_z[1]:.3f}"
        )
    if chassis_z[0] < -0.25:
        raise RuntimeError(
            f"Chassis still hangs below ground after Y-up map: zmin={chassis_z[0]:.3f}"
        )
    if left_y <= 0 or right_y >= 0:
        raise RuntimeError(
            f"Left/right swapped after Y-up map: left_y={left_y:.3f} right_y={right_y:.3f}"
        )

    sensor_z = max(mean_wheel_z + wheel_radius, chassis_z[1] - 0.05)
    print(
        f"Upright check OK: chassis z~[{chassis_z[0]:.3f},{chassis_z[1]:.3f}] "
        f"wheels z~{mean_wheel_z:.3f} r={wheel_radius:.3f} lidar z={sensor_z:.3f}"
    )

    meta = {
        "step_file": str(args.step),
        "tool": "cadquery-ocp (Open CASCADE 7.x) STEPControl + StlAPI",
        "grabcad_url": "https://grabcad.com/library/mercedes-amg-petronas-f1-concept-2",
        "cad_to_ros": "X=-Z_cad, Y=-X_cad, Z=+Y_cad (CAD Y-up → ROS Z-up, det>0)",
        "solid_count": len(solids),
        "wheelbase_m": args.wheelbase,
        "scale_factor": scale,
        "wheel_radius_m": wheel_radius,
        "wheel_width_m": wheel_width,
        "wheel_centers_ros_m": {
            "left_front": list(joint_positions["left_front_wheel"]),
            "right_front": list(joint_positions["right_front_wheel"]),
            "left_rear": list(joint_positions["left_rear_wheel"]),
            "right_rear": list(joint_positions["right_rear_wheel"]),
        },
        "assembly_bounds_ros_m": {"min": list(all_lo), "max": list(all_hi)},
        "chassis_z_m": {"min": chassis_z[0], "max": chassis_z[1]},
        "lidar_z_m": sensor_z,
        "ground_z_m": 0.0,
        "link_groups": {k: sorted(v) for k, v in link_groups.items()},
        "dropped_solids": {
            "indices": sorted(dropped),
            "reason": (
                "Leftover Formula 1 body solids at each corner (uprights / "
                "wishbones / brake ducts), ~140 mm inboard of the WHEEL tire+rim. "
                "Dropped so they do not poke out of the rubber."
            ),
        },
    }

    args.metadata.parent.mkdir(parents=True, exist_ok=True)
    args.metadata.write_text(json.dumps(meta, indent=2) + "\n")
    print(f"Wrote metadata {args.metadata}")
    print(f"Wheel radius ~{wheel_radius:.3f} m  width ~{wheel_width:.3f} m")

    if args.patch_xacro.is_file():
        patch_racecar_xacro(args.patch_xacro, joint_positions, sensor_z)
        print(f"Patched joints in {args.patch_xacro}")
    if args.patch_macros.is_file():
        patch_wheel_collision_macros(args.patch_macros, wheel_radius, wheel_width)
        print(f"Patched wheel collisions in {args.patch_macros}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
