from setuptools import setup
from glob import glob
package_name = 'ros2_arduino_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob("launch/*.launch.py")),
        ('share/' + package_name + "/params", glob("params/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2u',
    maintainer_email='ros2u@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_node = ros2_arduino_bridge.arduino_node:main'
        ],
    },
)
