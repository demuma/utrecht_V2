import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'remote'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='Chun Fei Lung',
    maintainer_email='chunfei.lung@hu.nl',
    description='Package for manual robotic arm control using a Logitech Dual Action gamepad.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remote = remote.remote:main'
        ],
    },
)
