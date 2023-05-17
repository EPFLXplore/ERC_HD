from setuptools import setup
from setuptools import find_packages

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('vision/' + package_name, [package_name+'/camera_projection.py']), # might not need this
        # ('vision' + package_name, [package_name+'/vision_publisher.py']),   # might not need this
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eylul',
    maintainer_email='eylulipci00@gmail.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = vision.test_ros:main',
            'listener = vision.fake_matthias:main'
        ],
    },
)
