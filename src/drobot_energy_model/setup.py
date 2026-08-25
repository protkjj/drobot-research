from setuptools import find_packages, setup

package_name = 'drobot_energy_model'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo11dk',
    maintainer_email='dongukleokim@gmail.com',
    description='INA226 energy logging and cost function modeling',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'energy_logger = drobot_energy_model.energy_logger:main',
        ],
    },
)
