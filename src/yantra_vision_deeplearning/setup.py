from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yantra_vision_deeplearning'

model_files = {}
for f in glob('models/**/*', recursive=True):
    if os.path.isfile(f):
        dest = os.path.join('share', package_name, os.path.dirname(f))
        model_files.setdefault(dest, []).append(f)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ] + list(model_files.items()),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yantra',
    maintainer_email='abyanraditya.rakapasha@gmail.com',
    description='Yantra deep learning vision package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balldetectrfdetr = yantra_vision_deeplearning.balldetectrfdetr:main',
            'openvino_detector = yantra_vision_deeplearning.openvino_detector:main',
        ],
    },
)
