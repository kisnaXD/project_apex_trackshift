#!/usr/bin/env python3
"""Put cone DAEs inside each cone model so gzclient never hits Fuel."""

from pathlib import Path
import shutil
import sys

CONES = {
    'blue_cone': 'cone_blue.dae',
    'yellow_cone': 'cone_yellow.dae',
    'orange_cone': 'cone.dae',
    'big_cone': 'cone_big.dae',
}


def patch(tracks_root: Path) -> None:
    meshes = tracks_root / 'meshes'
    models = tracks_root / 'models'
    for name, dae in CONES.items():
        src = meshes / dae
        dest_dir = models / name
        dest = dest_dir / dae
        if not src.is_file() or not dest_dir.is_dir():
            continue
        shutil.copy2(src, dest)
        sdf_path = dest_dir / 'model.sdf'
        text = sdf_path.read_text(encoding='utf-8')
        for old in (
            f'model://eufs_tracks/meshes/{dae}',
            f'file://{dae}',
        ):
            text = text.replace(old, f'model://{name}/{dae}')
        # Collision meshes force gzclient to import every DAE twice.
        text = text.replace(
            f"""      <collision name='collision'>
        <max_contacts>3</max_contacts>
        <geometry>
          <mesh>
            <uri>model://{name}/{dae}</uri>
          </mesh>
        </geometry>
      </collision>""",
            """      <collision name='collision'>
        <max_contacts>3</max_contacts>
        <geometry>
          <cylinder>
            <radius>0.11</radius>
            <length>0.32</length>
          </cylinder>
        </geometry>
      </collision>""",
        )
        sdf_path.write_text(text, encoding='utf-8')
        print(f'patched {sdf_path} -> model://{name}/{dae}')


if __name__ == '__main__':
    patch(Path(sys.argv[1]))
