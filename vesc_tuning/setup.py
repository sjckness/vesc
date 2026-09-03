import os
from glob import glob

from setuptools import setup

package_name = 'vesc_tuning'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('*.md')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreas Steffens',
    maintainer_email='andreas21steffens@gmail.com',
    description='Human-in-the-loop empirical VESC gain calibration nodes for F1TENTH.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'speed_tuning_node = vesc_tuning.speed_tuning_node:main',
            'steering_calibration_node = vesc_tuning.steering_calibration_node:main',
            'speed_sweep_diagnostic_node = vesc_tuning.speed_sweep_diagnostic_node:main',
        ],
    },
)
