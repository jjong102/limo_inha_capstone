from setuptools import find_packages, setup

package_name = 'limo_aj'

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
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_node     = limo_aj.traffic:main',
#            'line_follow_node = limo_aj.line_follow:main',
#            'rotary_node      = limo_aj.rotary:main',
            'cone_node        = limo_aj.cone:main',
            'cone_check_node        = limo_aj.cone_check:main',
            'lidar_check_node = limo_aj.lidar_check:main',
            'lidar_angle_node = limo_aj.lidar_angle_check:main',
            'steering_test_node = limo_aj.steering_test:main',

        ],
    },
)