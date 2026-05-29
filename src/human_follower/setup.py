from setuptools import find_packages, setup

package_name = 'human_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=[
        'test', 'test.*',
        'human_follower.dr_spaam', 'human_follower.dr_spaam.*',
    ]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/human_follower.launch.py', 'launch/delivery.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ROS2 Jazzy: DR-SPAAM lidar person detections + YOLO person tracker + follower.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dr_spaam_detector = human_follower.dr_spaam_detector_node:main',
            'yolo_person_tracker = human_follower.yolo_tracker_node:main',
            'human_follower = human_follower.follower_node:main',
            'delivery_manager = human_follower.delivery_manager:main',
            'scan_qos_relay = human_follower.scan_qos_relay:main',
            'map_explorer = human_follower.map_explorer:main',
            'cmd_vel_inverter = human_follower.cmd_vel_inverter:main',
        ],
    },
)
