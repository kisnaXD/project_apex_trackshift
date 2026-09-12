#!/usr/bin/env python3
"""Convert CC-BY F1 GLB into staged eufs_racecar Gazebo meshes (STL + DAE)."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import trimesh


def rot_glb_to_ros() -> np.ndarray:
    # GLB: X=width, Y=height, Z=length  ->  ROS: X=forward, Y=left, Z=up
    return trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0])


def mesh_from_nodes(scene: trimesh.Scene, nodes: list[str]) -> trimesh.Trimesh:
    meshes: list[trimesh.Trimesh] = []
    for node in nodes:
        transform, geom_name = scene.graph.get(node)
        if geom_name not in scene.geometry:
            continue
        mesh = scene.geometry[geom_name].copy()
        mesh.apply_transform(transform)
        meshes.append(mesh)
    if not meshes:
        raise RuntimeError(f'No geometry found for nodes: {nodes}')
    return trimesh.util.concatenate(meshes)


def subtree_nodes(scene: trimesh.Scene, root: str) -> list[str]:
    nodes = [root]
    stack = [root]
    while stack:
        parent = stack.pop()
        for child in scene.graph.transforms.children.get(parent, []):
            nodes.append(child)
            stack.append(child)
    return nodes


def split_lr(mesh: trimesh.Trimesh) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    verts = mesh.vertices
    faces = mesh.faces
    center_y = float(np.median(verts[:, 1]))
    left_mask = verts[:, 1] >= center_y
    right_mask = ~left_mask

    def submesh(mask: np.ndarray) -> trimesh.Trimesh:
        vert_ids = np.where(mask)[0]
        vert_map = -np.ones(len(verts), dtype=int)
        vert_map[vert_ids] = np.arange(len(vert_ids))
        keep = np.all(mask[faces], axis=1)
        sub_faces = vert_map[faces[keep]]
        return trimesh.Trimesh(vertices=verts[vert_ids], faces=sub_faces, process=True)

    left, right = submesh(left_mask), submesh(right_mask)
    if left.centroid[1] < right.centroid[1]:
        left, right = right, left
    return left, right


def export_pair(mesh: trimesh.Trimesh, stl_path: Path, dae_path: Path) -> None:
    stl_path.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(stl_path)
    mesh.export(dae_path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--glb', required=True)
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--target-length', type=float, default=5.0)
    parser.add_argument(
        '--promote',
        action='store_true',
        help='write directly to --out-dir instead of --out-dir/staged_f1_meshes',
    )
    parser.add_argument(
        '--replace-existing',
        action='store_true',
        help='when used with --promote, replace existing generated mesh files',
    )
    args = parser.parse_args()

    requested_out_dir = Path(args.out_dir)
    out_dir = requested_out_dir if args.promote else requested_out_dir / 'staged_f1_meshes'
    scene = trimesh.load(args.glb, force='scene')

    body_nodes = subtree_nodes(scene, 'car_body_5') + ['Object_4', 'Object_5']
    body = mesh_from_nodes(scene, body_nodes)
    front_wheels = mesh_from_nodes(scene, subtree_nodes(scene, 'front_wheels_7'))
    rear_wheels = mesh_from_nodes(scene, subtree_nodes(scene, 'back_wheels_1'))

    rot = rot_glb_to_ros()
    for m in (body, front_wheels, rear_wheels):
        m.apply_transform(rot)

    length = body.bounds[1][0] - body.bounds[0][0]
    scale = args.target_length / length
    center = (body.bounds[0] + body.bounds[1]) / 2.0
    center[2] = body.bounds[0][2]  # sit on ground (min Z)

    def normalize(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        mesh.vertices -= center
        mesh.vertices *= scale
        return mesh

    body = normalize(body)
    front_wheels = normalize(front_wheels)
    rear_wheels = normalize(rear_wheels)

    left_front, right_front = split_lr(front_wheels)
    left_rear, right_rear = split_lr(rear_wheels)

    wheel_radius = 0.33
    ground_z = float(left_rear.centroid[2] - wheel_radius)
    for m in (body, left_front, right_front, left_rear, right_rear):
        m.vertices[:, 2] -= ground_z

    y_mid = np.mean(
        [
            left_front.centroid[1],
            right_front.centroid[1],
            left_rear.centroid[1],
            right_rear.centroid[1],
        ]
    )
    for m in (body, left_front, right_front, left_rear, right_rear):
        m.vertices[:, 1] -= y_mid

    wheel_pairs = (
        ('left_front_wheel', left_front),
        ('right_front_wheel', right_front),
        ('left_rear_wheel', left_rear),
        ('right_rear_wheel', right_rear),
    )
    joint_positions: dict[str, list[float]] = {}
    for name, mesh in wheel_pairs:
        joint_positions[name] = mesh.centroid.tolist()
        mesh.vertices -= mesh.centroid

    generated_names = [
        'chassis.STL',
        'chassis.dae',
        'left_front_wheel.STL',
        'left_front_wheel.dae',
        'right_front_wheel.STL',
        'right_front_wheel.dae',
        'left_rear_wheel.STL',
        'left_rear_wheel.dae',
        'right_rear_wheel.STL',
        'right_rear_wheel.dae',
        'left_steering_hinge.STL',
        'left_steering_hinge.dae',
        'right_steering_hinge.STL',
        'right_steering_hinge.dae',
        'f1_mesh_meta.txt',
    ]
    existing_outputs = [name for name in generated_names if (out_dir / name).exists()]
    if existing_outputs and not (args.promote and args.replace_existing):
        raise RuntimeError(
            f'{out_dir} already has generated outputs: {existing_outputs}. '
            'Use a clean staging directory, or pass --promote --replace-existing.'
        )
    if args.promote and args.replace_existing:
        for name in generated_names:
            p = out_dir / name
            if p.exists():
                p.unlink()

    export_pair(body, out_dir / 'chassis.STL', out_dir / 'chassis.dae')
    for name, mesh in wheel_pairs:
        export_pair(mesh, out_dir / f'{name}.STL', out_dir / f'{name}.dae')

    # Tiny invisible steering hinge placeholders (ackermann uses joint, not mesh).
    hinge = trimesh.creation.box(extents=[0.01, 0.01, 0.01])
    export_pair(hinge, out_dir / 'left_steering_hinge.STL', out_dir / 'left_steering_hinge.dae')
    export_pair(hinge, out_dir / 'right_steering_hinge.STL', out_dir / 'right_steering_hinge.dae')

    meta = out_dir / 'f1_mesh_meta.txt'
    meta.write_text(
        '\n'.join(
            [
                f'target_length_m={args.target_length:.4f}',
                f'scale={scale:.6f}',
                f'body_bounds_min={body.bounds[0].tolist()}',
                f'body_bounds_max={body.bounds[1].tolist()}',
                f'left_front_joint={joint_positions["left_front_wheel"]}',
                f'right_front_joint={joint_positions["right_front_wheel"]}',
                f'left_rear_joint={joint_positions["left_rear_wheel"]}',
                f'right_rear_joint={joint_positions["right_rear_wheel"]}',
                f'wheel_radius_m={wheel_radius}',
                f'promoted={args.promote}',
                f'output_dir={out_dir}',
            ]
        )
        + '\n'
    )
    print(meta.read_text())


if __name__ == '__main__':
    main()
