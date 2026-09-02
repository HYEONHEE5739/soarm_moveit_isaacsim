from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'so_arm_gui_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'styles'), glob('so_arm_gui_control/styles/*.qss')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyeonhee',
    maintainer_email='lhh6225739@gmail.com',
    description='PyQt GUI for SOARM control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = so_arm_gui_control.gui_control_node:main'
        ],
    },
)
