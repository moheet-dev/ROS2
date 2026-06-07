from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot_services'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'srv'),
            glob('srv/*.srv')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moheet',
    maintainer_email='moheetprogrammer@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_service = turtlebot_services.move_service:main'
        ],
    },
)
