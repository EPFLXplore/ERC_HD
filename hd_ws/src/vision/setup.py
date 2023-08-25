from setuptools import setup

package_name = "vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, package_name + ".controlpanel", package_name + ".publishers", package_name + ".subscribers"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "img_publisher = vision.image_publisher:main",
            "img_subscriber = vision.subscribers.image_subscriber:main",
            "vision_node = vision.vision_node:main",
            "fake_task_selector = vision.publishers.fake_cs_task_selector:main",
        ],
    },
)
