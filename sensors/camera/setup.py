from setuptools import find_packages, setup
import os
from glob import glob

package_name = "camera"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),  # Ensure launch files are included
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xplore",
    maintainer_email="matthieu.andre@epfl.ch",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera = camera.camera_node:main",
            "depth = camera.depth_publisher:main",
            "bare_camera = camera.bare_camera_node:main",
            "marg_top_left = camera.nav_marg_cam:main"
        ],
    },
)
