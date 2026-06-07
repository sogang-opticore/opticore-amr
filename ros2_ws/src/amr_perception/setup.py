from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Perception package for Opticore AMR — YOLOv8 object detection',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector = amr_perception.yolo_detector:main',
            'object_tracker = amr_perception.object_tracker:main',
            'fused_tracker = amr_perception.fused_tracker:main',
        ],
    },
)
