import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'apriltag'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csl',
    maintainer_email='WillP_Li@compal.com',
    description='AprilTag visual tracker and controller node for ROS 2.',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'apriltag_detection = apriltag.detection_node:main',
            'apriltag_control = apriltag.control_node:main',
            'print_tag_pose = tools.print_tag_pose:main',
            'virtual_tracking_sim = tools.virtual_tracking_sim:main',
        ],
    },
)
