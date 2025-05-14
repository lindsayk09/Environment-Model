from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'environmental_model'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Ensure ROS 2 package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files (if any)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install config files (if any)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lindsay Shantha Rubia',
    maintainer_email='Lindsay-Shantha-Rubia.Kasthuri-Kalaimathi@stud.hs-coburg.de',
    description='Environmental Model for ROS 2 - Generates Grid Map from Lane, ODM, and Static Map Data',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'environment_node = environmental_model.environment_node:main',
        ],
    },
)
