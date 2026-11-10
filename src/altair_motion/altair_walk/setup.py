from setuptools import find_packages, setup

package_name = 'altair_walk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/altair_walk_sim.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'pinocchio',
        'xacro',
        'scikit-fuzzy',
        'control',
        'scipy',
        'pytransform3d',
        'matplotlib',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Altair',
    maintainer_email='altairgmrt@gmail.com',
    description='Altair walking engine with preview control, inverse kinematics, and fuzzy logic',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'altair_walk_node = altair_walk.walk:main',
            'enable_walk_module = altair_walk.enable_walk_module:main',
            'imu_relay = altair_walk.imu_relay:main',
            'heading_hold = altair_walk.heading_hold:main',
            'fall_detector = altair_walk.fall_detector:main',
            'joint_command_bridge = altair_walk.joint_command_bridge:main',
            'distance_tracker = altair_walk.distance_tracker:main',
            'robot_mode_controller = altair_walk.robot_mode_controller:main',
        ],
    },
)
