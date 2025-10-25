from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mybot_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raspberry',
    maintainer_email='raspberry@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node = mybot_calibration.calibration_node:main',
            'calibration_node_rotation = mybot_calibration.calibration_node_rotation:main'
        ],
    },
)
