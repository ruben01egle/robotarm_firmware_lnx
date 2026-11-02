from glob import glob

from setuptools import find_packages, setup

package_name = 'robotarm_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ruben01egle@gmail.com',
    description='Tools for working on the real arm',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'record = robotarm_tools.recorder:main',
            'motion_runner = robotarm_tools.motion_runner:main',
        ],
    },
)
