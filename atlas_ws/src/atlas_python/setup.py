from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'atlas_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join("share", package_name, "models"), glob("resource/models/*.pt")),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tyson',
    maintainer_email='tystruong@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actuation_test = atlas_python.actuation_test:main',
            'camera = atlas_python.camera:main',
            'encoder_test = atlas_python.encoder_test:main',
            'encoder = atlas_python.encoder:main',
            'hello = atlas_python.hello:main',
            'motor_driver = atlas_python.motor_driver:main',
            'object_detection = atlas_python.object_detection:main',
            'odometry = atlas_python.odometry:main'
            'viewer = atlas_python.viewer:main',
        ],
    },
)
