from setuptools import find_packages, setup
from glob import glob

package_name = 'colav_gateway'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.[pxy][mlo]*')),  # All launch files (.launch.py, .launch.xml, .launch.yaml)
        ('share/' + package_name + '/config', glob.glob('config/*')),  # All config files
        ('share/' + package_name + '/scripts', glob.glob('scripts/*')),  # All script files
        ('share/' + package_name + '/utils', glob.glob('utils/*')),  # All utility files
    ],
    install_requires=[
        'setuptools',
        'protobuf',
        'numpy',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='Ryan McKee',
    maintainer_email='r.mckee@qub.ac.uk',
    description=(
        'colav_gateway is the gateway to the colav system, providing nodes for managing '
        'a planning mission and receiving/providing feedback to external applications '
        'utilizing COLAV for autonomous navigation.'
    ),
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'colav_gateway_node = colav_gateway.colav_gateway:main'
        ],
    },
)
