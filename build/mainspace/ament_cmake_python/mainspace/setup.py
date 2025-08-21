from setuptools import find_packages
from setuptools import setup

setup(
    name='mainspace',
    version='0.0.0',
    packages=find_packages(
        include=('mainspace', 'mainspace.*')),
)
