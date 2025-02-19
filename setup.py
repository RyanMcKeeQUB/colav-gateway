from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'colav_gateway'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test', 'scripts', 'utils']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yml')),
        (f'lib/python3.10/site-packages/{package_name}/scripts', glob('scripts/*.py')), # adding scripts manually so theres an explicity namespace
        (f'lib/python3.10/site-packages/{package_name}/utils', glob('utils/*.py')), # adding utils manually so there is an explicit namespace in site-packages
    ],
    install_requires=[
        'setuptools',
        'protobuf',
        'numpy',
        'PyYAML',
        'colav-protobuf'
    ],
    zip_safe=True,
    maintainer='Ryan McKee',
    maintainer_email='r.mckee@qub.ac.uk',
    description=(
        'colav_gateway is the gateway to the colav system, providing nodes for managing '
        'a planning mission and receiving/providing feedback to external applications '
        'utilizing COLAV for autonomous navigation.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_interface_node = colav_gateway.execute_controller_interface_node:main',
            'mission_interface_node = colav_gateway.execute_mission_interface_node:main'
        ],
    },
)
