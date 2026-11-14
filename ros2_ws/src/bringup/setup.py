from glob import glob
from setuptools import setup

name = 'bringup'
setup(
    name=name, version='0.1.0', packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        ('share/' + name + '/launch', glob('launch/*.py')),
        ('share/' + name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ACAV team', maintainer_email='team@acav.local', license='MIT',
    description='Launch files and configuration for CARLA and WebDriveSim',
    entry_points={'console_scripts': ['carla_adapter = bringup.carla_adapter:main']},
)
