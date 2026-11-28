import os
from glob import glob

from setuptools import setup

package_name = 'yantra_heading'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yantra',
    maintainer_email='altairgmrt@gmail.com',
    description='IMU heading hold and closed-loop relative turns for the walk engines',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heading_controller = yantra_heading.heading_controller:main',
        ],
    },
)
