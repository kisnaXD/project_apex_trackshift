"""Generate RViz RobotModel displays for the bounded multi-car launch.

The helper is deliberately independent of RViz and ROS so ``lap_ros`` can
generate a per-run config before it starts RViz.  Each RSP publishes its
description under the car namespace; opponent TF frames carry the same
namespace as their RViz TF Prefix.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import math
from pathlib import Path

import yaml

from .grid_geometry import build_grid_poses


def car_namespace(index: int, namespace: str = 'eufs') -> str:
    """Return the public namespace used by load_car for a zero-based index."""
    if index < 0:
        raise ValueError('car index must be non-negative')
    base = namespace.strip('/') or 'eufs'
    return base if index == 0 else f'{base}{index + 1}'


def robot_model_display(index: int, namespace: str = 'eufs') -> dict:
    """Return one RViz RobotModel display for ``index``."""
    car = car_namespace(index, namespace)
    return {
        'Alpha': 1,
        'Class': 'rviz_default_plugins/RobotModel',
        'Collision Enabled': False,
        'Description File': '',
        'Description Source': 'Topic',
        'Description Topic': {
            'Depth': 5,
            'Durability Policy': 'Transient Local',
            'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable',
            'Value': f'/{car}/robot_description',
        },
        'Enabled': True,
        'Links': {
            'All Links Enabled': True,
            'Expand Joint Details': False,
            'Expand Link Details': False,
            'Expand Tree': False,
            'Link Tree Style': 'Links in Alphabetic Order',
        },
        'Mass Properties': {'Inertia': False, 'Mass': False},
        'Name': f'{car} RobotModel',
        # robot_state_publisher receives ``car/``; RViz's RobotModel adds
        # the separator itself, so its property must be the bare prefix.
        'TF Prefix': '' if index == 0 else car,
        'Update Interval': 0,
        'Value': True,
        'Visual Enabled': True,
    }


def generate_robot_model_displays(num_cars: int, namespace: str = 'eufs') -> list[dict]:
    """Generate exactly ``num_cars`` RobotModel display mappings (1..20)."""
    if not 1 <= int(num_cars) <= 20:
        raise ValueError('num_cars must be between 1 and 20')
    return [robot_model_display(index, namespace) for index in range(int(num_cars))]


# Short names keep the helper convenient for lap_ros callers while the longer
# names above remain self-documenting for direct imports and tests.
generate_grid_displays = generate_robot_model_displays


def generate_rviz_config(num_cars: int, namespace: str = 'eufs', base_config: dict | None = None) -> dict:
    """Return an RViz config with one generated RobotModel per car.

    Existing non-RobotModel displays (track markers, odometry, TF, and view)
    are preserved when ``base_config`` is supplied.
    """
    config = deepcopy(base_config) if base_config is not None else {
        'Visualization Manager': {'Class': '', 'Displays': [], 'Enabled': True},
    }
    manager = config.setdefault('Visualization Manager', {})
    existing = [display for display in manager.get('Displays', [])
                if display.get('Class') != 'rviz_default_plugins/RobotModel']
    manager['Displays'] = generate_robot_model_displays(num_cars, namespace) + existing
    return config


def fit_grid_view(config: dict, poses: list[tuple[float, float, float]]) -> dict:
    """Fit the generated multi-car poses while preserving the authored N=1 view."""
    if len(poses) <= 1:
        return config
    views = config.setdefault('Visualization Manager', {}).setdefault('Views', {})
    current = views.setdefault('Current', {})
    min_x = min(pose[0] for pose in poses)
    max_x = max(pose[0] for pose in poses)
    min_y = min(pose[1] for pose in poses)
    max_y = max(pose[1] for pose in poses)
    span = max(max_x - min_x, max_y - min_y, 1.0)
    focal = current.setdefault('Focal Point', {})
    focal['X'] = 0.5 * (min_x + max_x)
    focal['Y'] = 0.5 * (min_y + max_y)
    focal.setdefault('Z', 0.4)
    current['Distance'] = max(float(current.get('Distance', 32.0)), span * 1.8)
    for saved in views.get('Saved', []):
        if isinstance(saved, dict):
            saved['Focal Point'] = deepcopy(focal)
            saved['Distance'] = current['Distance']
    return config


build_rviz_grid = generate_rviz_config


def write_rviz_grid(output: str | Path, num_cars: int, namespace: str = 'eufs', base_config: dict | None = None, assets: dict | None = None) -> Path:
    """Write a generated config and return its path."""
    path = Path(output)
    config = generate_rviz_config(num_cars, namespace, base_config)
    if assets is not None and int(num_cars) > 1:
        config = fit_grid_view(config, build_grid_poses(assets, int(num_cars)))
    path.write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cars', type=int, default=1)
    parser.add_argument('--namespace', default='eufs')
    parser.add_argument('--base-config', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    base = yaml.safe_load(args.base_config.read_text(encoding='utf-8')) if args.base_config else None
    write_rviz_grid(args.output, args.cars, args.namespace, base)


if __name__ == '__main__':
    main()
