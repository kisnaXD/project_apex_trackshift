from setuptools import setup

package_name = "eufs_race_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["eufs_race_gui = eufs_race_gui.cli:main"]},
    zip_safe=True,
    description="Headless-safe telemetry, decision and replay GUI for EUFS race controls",
    license="MIT",
)

