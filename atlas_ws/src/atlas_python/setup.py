from setuptools import find_packages, setup

package_name = 'atlas_python'

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
    maintainer='tyson',
    maintainer_email='tystruong@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'atlas_test_node = atlas_python.atlas_test_node:main',
            'actuation = atlas_python.actuation:main'
        ],
    },
)
