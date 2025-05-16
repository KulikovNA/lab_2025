from setuptools import setup
import os
from glob import glob

package_name = 'cv_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kulikov',
    maintainer_email='kulikov@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['module_ROS = cv_basics.module_ROS:main',
                            'service = cv_basic.service',
                            'inference_yolo_node = cv_basics.inference_pytorch_background:main',
                            'scan_subscriber = cv_basics.scan_subscriber:main'
        ],
    },
)
