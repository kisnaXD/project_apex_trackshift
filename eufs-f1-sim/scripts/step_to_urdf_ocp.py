#!/usr/bin/env python3
"""
Export GrabCAD Mercedes F1 STEP assembly to per-link STL meshes for eufs_racecar.

Uses Open CASCADE (cadquery-ocp) for STEP import and STL tessellation.
Solid grouping is derived from assembly bounding boxes (see classify_solids()).
"""
from __future__ import annotations

import argparse
import json
import sys
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

# GrabCAD "Assem step.STEP" solid indices (33 solids total).
CHASSIS_BODY = {24}
CHASSIS_DETAIL = set(range(0, 20))
FRONT_WING = {28, 32}
REAR_WING = {26, 30}
LEFT_FRONT_WHEEL = {22, 27}
RIGHT_FRONT_WHEEL = {23, 31}
LEFT_REAR_WHEEL = {20, 25}
RIGHT_REAR_WHEEL = {21, 29}

LINK_GROUPS = {
    "chassis": CHASSIS_BODY | CHASSIS_DETAIL,
    "front_wing": FRONT_WING,
    "rear_wing": REAR_WING,
    "left_front_wheel": LEFT_FRONT_WHEEL,
    "right_front_wheel": RIGHT_FRONT_WHEEL,
    "left_rear_wheel": LEFT_REAR_WHEEL,
    "right_rear_wheel": RIGHT_REAR_WHEEL,
}

TARGET_WHEELBASE_M = 3.28
MESH_DEFLECTION_MM = 0.8


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


def solid_center_mm(solid) -> tuple[float, float, float]:
    bb = Bnd_Box()
    BRepBndLib.Add_s(solid, bb)
    xmin, ymin, zmin, xmax, ymax, zmax = bb.Get()
    return ((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2)


def cad_mm_to_ros_m(x: float, y: float, z: float, scale: float) -> tuple[float, float, float]:
    """CAD: Z forward (low=front), X left, Y down  ->  ROS: X forward, Y left, Z up (m)."""
    return (-z * scale, x * scale, -y * scale)


def make_compound(solids: list) -> TopoDS_Compound:
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    for solid in solids:
        builder.Add(comp, solid)
    return comp


def bounds_ros(solids: list, scale: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    xs, ys, zs = [], [], []
    for solid in solids:
        bb = Bnd_Box()
        BRepBndLib.Add_s(solid, bb)
        xmin, ymin, zmin, xmax, ymax, zmax = bb.Get()
        for x, y, z in (
            (xmin, ymin, zmin),
            (xmax, ymin, zmin),
            (xmin, ymax, zmin),
            (xmax, ymax, zmin),
            (xmin, ymin, zmax),
            (xmax, ymin, zmax),
            (xmin, ymax, zmax),
            (xmax, ymax, zmax),
        ):
            rx, ry, rz = cad_mm_to_ros_m(x, y, z, scale)
            xs.append(rx)
            ys.append(ry)
            zs.append(rz)
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def wheelbase_mm(solids: list) -> float:
    front_z = sum(solid_center_mm(solids[i])[2] for i in LEFT_FRONT_WHEEL | RIGHT_FRONT_WHEEL) / 4
    rear_z = sum(solid_center_mm(solids[i])[2] for i in LEFT_REAR_WHEEL | RIGHT_REAR_WHEEL) / 4
    return rear_z - front_z


def ros_transform(scale: float) -> gp_Trsf:
    """Uniform scale plus CAD->ROS axis remap."""
    tr = gp_Trsf()
    tr.SetValues(
        0, 0, -scale, 0,
        scale, 0, 0, 0,
        0, -scale, 0, 0,
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


def group_center_ros(solids: list, indices: set[int], scale: float) -> tuple[float, float, float]:
    pts = [cad_mm_to_ros_m(*solid_center_mm(solids[i]), scale) for i in indices]
    n = len(pts)
    return (sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n, sum(p[2] for p in pts) / n)


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
    parser.add_argument("--metadata", type=Path, default=Path("overlay/eufs_racecar/eufs_racecar/meshes/step_export.json"))
    args = parser.parse_args()

    if not args.step.is_file():
        print(f"STEP not found: {args.step}", file=sys.stderr)
        return 1

    solids = read_solids(args.step)
    print(f"Read {len(solids)} solids from {args.step.name}")

    wb_mm = wheelbase_mm(solids)
    scale = args.wheelbase / wb_mm  # metres per millimetre
    print(f"Wheelbase CAD {wb_mm:.1f} mm -> scale {scale:.6f} m/mm (target {args.wheelbase} m)")

    tr = ros_transform(scale)

    lf = group_center_ros(solids, LEFT_FRONT_WHEEL, scale)
    rf = group_center_ros(solids, RIGHT_FRONT_WHEEL, scale)
    lr = group_center_ros(solids, LEFT_REAR_WHEEL, scale)
    rr = group_center_ros(solids, RIGHT_REAR_WHEEL, scale)
    rear_mid = (
        (lr[0] + rr[0]) / 2,
        (lr[1] + rr[1]) / 2,
        (lr[2] + rr[2]) / 2,
    )
    all_lo_pre, _ = bounds_ros([solids[i] for i in range(len(solids))], scale)
    ground_z = all_lo_pre[2]
    # EUFS chassis frame: X forward, rear axle at x=0, lowest vertex on z=0.
    shift = gp_Vec(-rear_mid[0], -rear_mid[1], -ground_z)
    tr_pre = gp_Trsf()
    tr_pre.SetTranslation(shift)
    tr = tr_pre.Multiplied(tr)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    for link, indices in LINK_GROUPS.items():
        members = [solids[i] for i in sorted(indices)]
        compound = make_compound(members)
        out = args.out_dir / f"{link}.STL"
        export_stl(compound, out, tr, MESH_DEFLECTION_MM)
        lo, hi = bounds_ros(members, scale)
        print(f"  {link}: {out.name} ({out.stat().st_size // 1024} KiB)")

    # Tiny steering hinge placeholders (ackermann expects the links).
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox

    box = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape()
    for name in ("left_steering_hinge", "right_steering_hinge"):
        export_stl(box, args.out_dir / f"{name}.STL", gp_Trsf(), 0.5)

    def center_after_shift(indices: set[int]) -> tuple[float, float, float]:
        c = group_center_ros(solids, indices, scale)
        return (c[0] + shift.X(), c[1] + shift.Y(), c[2] + shift.Z())

    lf = center_after_shift(LEFT_FRONT_WHEEL)
    rf = center_after_shift(RIGHT_FRONT_WHEEL)
    lr = center_after_shift(LEFT_REAR_WHEEL)
    rr = center_after_shift(RIGHT_REAR_WHEEL)

    all_lo, all_hi = bounds_ros([solids[i] for i in range(len(solids))], scale)
    all_lo = (all_lo[0] + shift.X(), all_lo[1] + shift.Y(), all_lo[2] + shift.Z())
    all_hi = (all_hi[0] + shift.X(), all_hi[1] + shift.Y(), all_hi[2] + shift.Z())
    ground_z = 0.0

    meta = {
        "step_file": str(args.step),
        "tool": "cadquery-ocp (Open CASCADE 7.x) STEPControl + StlAPI",
        "grabcad_url": "https://grabcad.com/library/mercedes-amg-petronas-f1-concept-2",
        "solid_count": len(solids),
        "wheelbase_m": args.wheelbase,
        "scale_factor": scale,
        "wheel_centers_ros_m": {
            "left_front": lf,
            "right_front": rf,
            "left_rear": lr,
            "right_rear": rr,
        },
        "assembly_bounds_ros_m": {"min": all_lo, "max": all_hi},
        "ground_z_m": ground_z,
        "link_groups": {k: sorted(v) for k, v in LINK_GROUPS.items()},
    }

    args.metadata.parent.mkdir(parents=True, exist_ok=True)
    args.metadata.write_text(json.dumps(meta, indent=2) + "\n")
    print(f"Wrote metadata {args.metadata}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
