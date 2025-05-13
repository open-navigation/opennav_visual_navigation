from glob import glob
import os

from setuptools import setup

package_name = 'opennav_visual_nav_demo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('params/*')),
        (os.path.join('share', package_name), glob('behavior_tree/*')),
        (os.path.join('share', package_name), glob('tools/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve macenski',
    maintainer_email='steve@opennav.org',
    description='A visual navigation demo using NVIDIA Jetson, and Isaac SDKs',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_application = opennav_visual_nav_demo.patrol_application:main',
        ],
    },
)
