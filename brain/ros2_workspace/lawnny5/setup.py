from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lawnny5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'config'), glob('config/*', recursive=True)),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*', recursive=True)),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*', recursive=True)),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_settings_server = lawnny5.global_settings_server:main',
            'motor_control_multiplexer = lawnny5.motor_control_multiplexer:main',
            'sabertooth_mixed_controller = lawnny5.sabertooth_mixed_controller:main',
            'sabertooth_driver = lawnny5.sabertooth_driver:main',
            'depth_ai_camera = lawnny5.depth_ai_camera:main',
            'depth_ai_camera_uvc = lawnny5.depth_ai_camera_uvc:main',
            'web_control_ui = lawnny5.web_control_ui:main',
            'webots_driver = lawnny5.webots_driver:main',
        ],
    },
)
