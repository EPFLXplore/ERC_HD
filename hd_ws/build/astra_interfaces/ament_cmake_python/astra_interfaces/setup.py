from setuptools import find_packages
from setuptools import setup

setup(
    name='astra_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('astra_interfaces', 'astra_interfaces.*')),
)
