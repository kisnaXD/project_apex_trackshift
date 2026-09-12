import os
from glob import glob

from setuptools import setup

package_name = 'eufs_racecar'


def package_files(directory):
    paths = []
    for path, _, filenames in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


setup(
    name=package_name,
    version='2.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'robots', 'eufs'), glob('robots/eufs/*')),
        (os.path.join('share', package_name, 'urdf'), glob('eufs_racecar/urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('eufs_racecar/meshes/*')),
        (os.path.join('share', package_name, 'materials', 'scripts'), glob('eufs_racecar/materials/scripts/*')),
    ] + [
        (os.path.join('share', package_name, 'models', os.path.relpath(os.path.dirname(path), 'eufs_racecar/models')), [path])
        for path in package_files('eufs_racecar/models')
    ] if os.path.isdir('eufs_racecar/models') else [],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'track_marker_publisher = eufs_racecar.track_marker_publisher:main',
            'start_dashboard = eufs_racecar.start_dashboard:main',
            'tyre_state_publisher = eufs_racecar.tyre_state_publisher:main',
            'ackermann_cmd_bridge = eufs_racecar.ackermann_cmd_bridge:main',
            'odom_tf_publisher = eufs_racecar.odom_tf_publisher:main',
            'rviz_grid = eufs_racecar.rviz_grid:main',
        ],
    },
    zip_safe=True,
    maintainer='EUFS F1 Sim',
    maintainer_email='eufs-f1-sim@local',
    description='Open-license F1 visual on eufs_racecar ackermann stack for EUFS sim',
    license='MIT',
)
