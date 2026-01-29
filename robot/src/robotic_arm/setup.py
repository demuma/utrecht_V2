import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robotic_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Chun Fei Lung',
    maintainer_email='chunfei.lung@hu.nl',
    description='Driver package for the prototype version of HU University of Applied Sciences Utrecht’s remote-controlled robotic arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = robotic_arm.driver:main',
            'telemetry = robotic_arm.telemetry:main'
        ],
    },
)
