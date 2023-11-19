import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'auto_car'
lib = 'auto_car/lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,lib],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'),
        glob('launch/*')),
         (os.path.join('share', package_name, 'scripts'),
        glob('scripts/*')),
(os.path.join('share', package_name, 'config'), ['.env']),
(os.path.join('share', package_name, 'scripts'), ['.env'])
    ],

    install_requires=['setuptools','python-socketio','python-dotenv','pyserial'],
    zip_safe=True,
    maintainer='nkcong206',
    maintainer_email='nkcong206@gmail.com',
    description='autonomous car',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planning = auto_car.planning_node:main',
            'planning_test = auto_car.planning_node_test:main',
            'socketio = auto_car.socketio_node:main',
            'gps = auto_car.gps_node:main',
            'gps_test = auto_car.gps_node_test:main',
            'controller = auto_car.controller_node:main',
        ],
    },
)
