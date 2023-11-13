from setuptools import find_packages, setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nkcong206',
    maintainer_email='nkcong206@gmail.com',
    description='ROS2 controller',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = controller.controller:main',
            'socketio = controller.socketio_node:main',
            'gps = controller.gps:main',
            'led = controller.led_display:main',
        ],
    },
)
