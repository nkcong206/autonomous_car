from setuptools import find_packages, setup

package_name = 'controller'
src = "controller/src"
config = "controller/config"
scripts = "controller/scripts"
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, src, config, scripts],
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
            'planning = controller.planning_final:main',
            'socketio = controller.socketio_final:main',
            'gps = controller.gps_final:main',
            'controller = controller.controller_final:main',
        ],
    },
)
