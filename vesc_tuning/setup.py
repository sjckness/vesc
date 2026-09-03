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
    # Declares the 'test' extra colcon's ament_python test step looks for
    # before it will invoke pytest at all. Without it colcon falls back to
    # `setup.py test`, whose unittest discovery finds none of this package's
    # pytest-style tests and reports "Ran 0 tests ... OK" -- a green result
    # that ran nothing. Not tests_require: modern setuptools does not
    # recognize that argument (it warns and ignores it) and it never enabled
    # pytest either.
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'speed_tuning_node = vesc_tuning.speed_tuning_node:main',
            'steering_calibration_node = vesc_tuning.steering_calibration_node:main',
            'speed_sweep_diagnostic_node = vesc_tuning.speed_sweep_diagnostic_node:main',
        ],
    },
)
