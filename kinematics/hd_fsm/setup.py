from setuptools import setup

package_name = "hd_fsm"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, package_name + "/interfaces"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="matthias",
    maintainer_email="matthias.schuller@epfl.ch",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fsm = hd_fsm.fsm:main",
            "fsm_servo = hd_fsm.fsm_servo:main",
            "simple_fsm = hd_fsm.clean_fsm:main",
        ],
    },
)
