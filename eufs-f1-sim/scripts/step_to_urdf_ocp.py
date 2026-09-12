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
import re
import sys
from dataclasses import dataclass
from pathlib import Path

from OCP.BRep import BRep_Builder
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS_Compound
from OCP.gp import gp_Pnt, gp_Trsf, gp_Vec

TARGET_WHEELBASE_M = 3.28
MESH_DEFLECTION_MM = 0.8
TINY_VOLUME_MM3 = 50_000.0
WING_VOLUME_MM3 = 1_000_000.0

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


def classify_solids(solids: list) -> dict[str, set[int]]:
    """Group STEP solids into chassis, wings, and four corner wheels."""
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

    for info in candidates:
        if info.volume_mm3 < TINY_VOLUME_MM3:
            chassis.add(info.index)
            continue

        cx, _, cz = info.center_mm
        is_front = cz < mid_z

        if info.volume_mm3 >= WING_VOLUME_MM3:
            groups["front_wing" if is_front else "rear_wing"].add(info.index)
            continue

        # Y_ros = -X_cad, so CAD +X (left in the STEP file) is ROS right.
        if is_front:
            groups["right_front_wheel" if cx > 0 else "left_front_wheel"].add(info.index)
        else:
            groups["right_rear_wheel" if cx > 0 else "left_rear_wheel"].add(info.index)

    groups["chassis"] = chassis

    assigned = set().union(*groups.values())
    if assigned != set(range(len(solids))):
        missing = set(range(len(solids))) - assigned
        groups["chassis"].update(missing)

    for name in WHEEL_LINKS:
        if not groups[name]:
            raise RuntimeError(f"No solids classified for {name}")

    return groups


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


def transform_points(points: list[tuple[float, float, float]], tr: gp_Trsf) -> list[tuple[float, float, float]]:
    transformed = []
    for x, y, z in points:
        p = gp_Pnt(x, y, z)
        p.Transform(tr)
        transformed.append((p.X(), p.Y(), p.Z()))
    return transformed


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


def export_stl(shape, out_path: Path, transform: gp_Trsf, deflection_mm: float) -> None:
    transformed = BRepBuilderAPI_Transform(shape, transform, True).Shape()
    mesh = BRepMesh_IncrementalMesh(transformed, deflection_mm, False, 0.5, True)
    mesh.Perform()
    if not mesh.IsDone():
        raise RuntimeError(f"meshing failed for {out_path.name}")
    writer = StlAPI_Writer()
    if not writer.Write(transformed, str(out_path)):
        raise RuntimeError(f"STL write failed: {out_path}")


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


def wheel_radius_m(
    solids: list,
    indices: set[int],
    scale: float,
    shift: tuple[float, float, float],
) -> float:
    members = [solids[i] for i in indices]
    center = group_center_shifted(solids, indices, scale, shift)
    points = corners_shifted(members, scale, shift)
    return max(
        ((p[0] - center[0]) ** 2 + (p[2] - center[2]) ** 2) ** 0.5
        for p in points
    )


def patch_wheel_collision_macros(macros_path: Path, wheel_radius: float) -> None:
    text = macros_path.read_text(encoding="utf-8")
    cyl_pattern = (
        r'(<xacro:macro name="(?:left|right)_wheels_collision_geometry">\s*'
        r'<origin xyz="0 0 0" rpy="0 1\.5708 0" />\s*'
        r'<geometry>\s*'
        r'<cylinder length=")([0-9.]+)(" radius=")([0-9.]+)(" />)'
    )
    cyl_repl = rf'\g<1>{2 * wheel_radius:.3f}\g<3>{wheel_radius:.3f}\g<5>'
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

    link_groups = classify_solids(solids)
    for name in LINK_NAMES:
        print(f"  {name}: {sorted(link_groups[name])}")

    wb_mm = wheelbase_mm(solids, link_groups)
    scale = args.wheelbase / wb_mm
    print(f"Wheelbase CAD {wb_mm:.1f} mm -> scale {scale:.6f} m/mm (target {args.wheelbase} m)")

    tr = ros_transform(scale)

    lr = group_center_ros(solids, link_groups["left_rear_wheel"], scale)
    rr = group_center_ros(solids, link_groups["right_rear_wheel"], scale)
    rear_mid = ((lr[0] + rr[0]) / 2, (lr[1] + rr[1]) / 2, (lr[2] + rr[2]) / 2)

    wheel_members = [solids[i] for name in WHEEL_LINKS for i in link_groups[name]]
    wheel_points = corners_shifted(wheel_members, scale, (0.0, 0.0, 0.0))
    ground_z = z_extent(wheel_points)[0]

    shift_tuple = (-rear_mid[0], -rear_mid[1], -ground_z)
    shift = gp_Vec(*shift_tuple)
    tr_pre = gp_Trsf()
    tr_pre.SetTranslation(shift)
    tr = tr_pre.Multiplied(tr)

    args.out_dir.mkdir(parents=True, exist_ok=True)

    joint_positions: dict[str, tuple[float, float, float]] = {}
    wheel_radii: list[float] = []

    for link in LINK_NAMES:
        members = [solids[i] for i in sorted(link_groups[link])]
        compound = make_compound(members)
        out = args.out_dir / f"{link}.STL"

        if link in WHEEL_LINKS:
            center = group_center_shifted(solids, link_groups[link], scale, shift_tuple)
            joint_positions[link] = center
            wheel_radii.append(wheel_radius_m(solids, link_groups[link], scale, shift_tuple))
            local_shift = gp_Trsf()
            local_shift.SetTranslation(gp_Vec(-center[0], -center[1], -center[2]))
            export_tr = local_shift.Multiplied(tr)
        else:
            export_tr = tr

        export_stl(compound, out, export_tr, MESH_DEFLECTION_MM)
        z_lo, z_hi = z_extent(corners_shifted(members, scale, shift_tuple))
        print(f"  {link}: {out.name} ({out.stat().st_size // 1024} KiB) z~[{z_lo:.3f},{z_hi:.3f}]")

    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox

    box = BRepPrimAPI_MakeBox(0.001, 0.001, 0.001).Shape()
    for name in ("left_steering_hinge", "right_steering_hinge"):
        export_stl(box, args.out_dir / f"{name}.STL", gp_Trsf(), 0.5)

    wheel_radius = sum(wheel_radii) / len(wheel_radii)
    all_lo, all_hi = bounds_ros([solids[i] for i in range(len(solids))], scale)
    all_lo = apply_shift(all_lo, shift_tuple)
    all_hi = apply_shift(all_hi, shift_tuple)

    chassis_z = z_extent(corners_shifted(
        [solids[i] for i in link_groups["chassis"]], scale, shift_tuple
    ))
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
        f"wheels z~{mean_wheel_z:.3f} lidar z={sensor_z:.3f}"
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
    }

    args.metadata.parent.mkdir(parents=True, exist_ok=True)
    args.metadata.write_text(json.dumps(meta, indent=2) + "\n")
    print(f"Wrote metadata {args.metadata}")
    print(f"Wheel radius ~{wheel_radius:.3f} m")

    if args.patch_xacro.is_file():
        patch_racecar_xacro(args.patch_xacro, joint_positions, sensor_z)
        print(f"Patched joints in {args.patch_xacro}")
    if args.patch_macros.is_file():
        patch_wheel_collision_macros(args.patch_macros, wheel_radius)
        print(f"Patched wheel collisions in {args.patch_macros}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
