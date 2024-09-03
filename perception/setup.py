from setuptools import find_packages, setup
import os
from glob import glob

package_name = "perception"

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
    maintainer="matthieu",
    maintainer_email="matthieu.andre@epfl.ch",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gui_node = perception.gui_node:main",
            "perception_node = perception.perception_node:main",
            "button_press_server = perception.gui_node_button_press_service_test:main",
            "model_server = perception.model_server:main",
            "test_server = perception.test.test_server:main",
            "model_node = perception.model_node:main",
        ],
    },
)
