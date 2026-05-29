from setuptools import find_packages, setup

package_name = 'first_scan'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/first_scan.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
            'config/explore_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='daniel@local',
    description='First-scan orchestrator: nav2 + explore_lite + save_map.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_and_exit = first_scan.save_and_exit:main',
            'pre_explore_rotate = first_scan.pre_explore_rotate:main',
        ],
    },
)
