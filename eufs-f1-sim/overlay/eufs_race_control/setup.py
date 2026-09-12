from glob import glob
import os

from setuptools import find_packages, setup

setup(
    name="eufs_race_control",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/eufs_race_control"]),
        ("share/eufs_race_control", ["package.xml"]),
        (os.path.join("share", "eufs_race_control", "config"), glob("config/*.yaml")),
    ],
    scripts=["scripts/run_quick_overtake.py"],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Typed EUFS race control foundation",
    license="MIT",
)
