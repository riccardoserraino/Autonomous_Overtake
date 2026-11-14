from glob import glob
from setuptools import setup

name = 'acc_lane_keeping'
setup(
    name=name, version='0.1.0', packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ACAV team', maintainer_email='team@acav.local', license='MIT',
    description='Adaptive cruise control + lane keeping',
    entry_points={'console_scripts': ['acc_node = acc_lane_keeping.acc_node:main']},
)
