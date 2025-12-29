from setuptools import find_packages
from setuptools import setup

setup(
    name='swarm_drone_control',
    version='0.1.0',
    packages=find_packages(
        include=('swarm_drone_control', 'swarm_drone_control.*')),
)
