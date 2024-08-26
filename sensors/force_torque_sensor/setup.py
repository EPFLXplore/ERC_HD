from setuptools import find_packages, setup

package_name = "force_torque_sensor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nina",
    maintainer_email="nina.lahellec@epfl.ch",
    description="Examples of minimal publisher/subscriber using rclpy",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "talker = force_torque_sensor.publisher_member_function:main",
            "listener = force_torque_sensor.subscriber_member_function:main",
        ],
    },
)
