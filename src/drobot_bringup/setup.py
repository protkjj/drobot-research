import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'drobot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config', 'common'), glob('config/common/*')),
        (os.path.join('share', package_name, 'config', 'navigation'), glob('config/navigation/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo11dk',
    maintainer_email='dongukleokim@gmail.com',
    description='Drobot bringup + UI launcher',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'start_goal_markers = drobot_bringup.start_goal_markers:main',
        ],
    },
)
