from setuptools import find_packages, setup

package_name = 'limo_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wego',
    maintainer_email='wego@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stop_and_go = limo_lidar.stop_and_go:main',
            'obstacle_avoid=limo_lidar.obstacle_avoid:main',
            'parking=limo_lidar.parking:main',
            'test=limo_lidar.test:main',
            'reset_steer=limo_lidar.reset_steer:main',
            'yellow_det=limo_lidar.yellow_det:main',
            'line_check=limo_lidar.line_check:main',
            'stop_test=limo_lidar.stop_test:main',
            'stop_test_2=limo_lidar.stop_test_2:main',
            'lidar_parking=limo_lidar.lidar_parking:main',
        ],
    },
)
