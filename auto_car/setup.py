from setuptools import find_packages, setup

package_name = 'auto_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nkcong206',
    maintainer_email='nkcong206@gmail.com',
    description='auto_car',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planning = auto_car.planning_final:main',
            'socketio = auto_car.socketio_final:main',
            'gps = auto_car.gps_final:main',
            'controller = auto_car.controller_final:main',
        ],
    },
)
