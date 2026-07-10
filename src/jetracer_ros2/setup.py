from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jetracer_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'),
            glob(os.path.join('params', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wego',
    maintainer_email='wego@todo.todo',
    description='JetRacer-based imitation learning nodes for LIMO',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collection_node = jetracer_ros2.data_collection_node:main',
            'inference_node = jetracer_ros2.inference_node:main',
            'inference_StopAndGo_with_joy_node = jetracer_ros2.inference_stopandgo_with_joy_node:main',
            'inference_StopAndGo_with_lidar_node = jetracer_ros2.inference_stopandgo_with_lidar_node:main',
            'inference_people_estop_node = jetracer_ros2.inference_people_estop_node:main',
            'mission_inference_node = jetracer_ros2.mission_inference_node:main',
            'mission_manager_node = jetracer_ros2.mission_manager_node:main',
            'mission_traffic_light_node = jetracer_ros2.mission_traffic_light_node:main',
            'debug_mission_traffic_light_node = jetracer_ros2.debug_mission_traffic_light_node:main',
            'mission_tunnel_node = jetracer_ros2.mission_tunnel_node:main',
            'mission_manager_tunnel_test_node = jetracer_ros2.mission_manager_tunnel_test_node:main',
            'mission_people_estop_node = jetracer_ros2.mission_people_estop_node:main',
            'debug_mission_people_estop_node = jetracer_ros2.debug_mission_people_estop_node:main',
            'mission_manager_pestop_test_node = jetracer_ros2.mission_manager_pestop_test_node:main',
            'inference_go_stop_node = jetracer_ros2.inference_go_stop_node:main',
            'mission_track_following_node = jetracer_ros2.mission_track_following_node:main',
            'mission_manager_track_following_test_node = jetracer_ros2.mission_manager_track_following_test_node:main',
        ],
    },
)
