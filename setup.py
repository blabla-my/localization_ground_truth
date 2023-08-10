import os
from glob import glob
from setuptools import setup

package_name = 'localization_ground_truth'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MingyuanLuo',
    maintainer_email='myluo22@m.fudan.edu.cn',
    description='Localization ground truth publisher for carla and autoware',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localizer = localization_ground_truth.localization_ground_truth:main'
        ],
    },
)
