from setuptools import find_packages, setup

package_name = 'altair_turn'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/altair_turn_sim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Altair',
    maintainer_email='altairgmrt@gmail.com',
    description='Independent turn layer for Altair',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turn_supervisor = altair_turn.turn_supervisor:main',
            'cmd_mux = altair_turn.cmd_mux:main',
        ],
    },
)
