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


mesh_files = package_files('eufs_racecar/meshes') if os.path.isdir('eufs_racecar/meshes') else []
model_files = package_files('eufs_racecar/models') if os.path.isdir('eufs_racecar/models') else []

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
    ] + [
        (os.path.join('share', package_name, os.path.dirname(path)), [path])
        for path in mesh_files + model_files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EUFS F1 Sim',
    maintainer_email='eufs-f1-sim@local',
    description='MIT racecar as eufs_racecar for EUFS sim lean slice',
    license='MIT',
)
