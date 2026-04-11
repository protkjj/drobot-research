from setuptools import find_packages, setup

package_name = 'drobot_mode_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/mode_switch_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo11dk',
    maintainer_email='dongukleokim@gmail.com',
    description='Mode switch manager: subscribe /mode_switch_points, command PX4 takeoff/landing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
