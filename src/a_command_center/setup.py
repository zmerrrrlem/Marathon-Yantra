from setuptools import find_packages, setup

package_name = 'a_command_center'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/execute_ukf.launch.py']),
        ('share/' + package_name + '/config', ['config/ukf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Altair',
    maintainer_email='altairgmrt@gmail.com',
    description='Altair command center - behavior trees, UKF fusion, and motion launch coordination',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
