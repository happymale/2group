"""HEVEN 2026 MIT base code bringup: localization -> planner -> control (Stanley + PI) and the vehicle's sensor TF.

    ros2 launch hk_mit bringup.launch.py
Inside the participant image (./run.sh shell, in /mit_ws). The course files are this package's course/ folder
(route.csv, course.json; the simulator also writes the same files per attempt to /data/current/).
Arguments: target_speed (m/s, default 5.0), rviz:=true (RViz: in the ./run.sh desktop), perception:=true (also start the LiDAR
perception nodes, which only subscribe), sensor_tf:=false (do not publish the sensor TF).
Each node can be replaced by your own (Python or C++) as long as it uses the same topics:
    /hk/local/odom (nav_msgs/Odometry), /hk/plan/local_path (nav_msgs/Path), /hk/control/command.

Sensor TF: config/sensor_mounts.yaml (sensor positions from the rear axle centre) becomes a fixed-joint robot model
published by robot_state_publisher on /tf_static: base_link -> rear_axle_link -> gps_link, imu_link, lidar_link.
localization.py publishes map -> base_link.
"""
import math
import os
from xml.etree import ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'hk_mit'


def robot_description(mounts):
    """Fixed-joint URDF from the sensor mounts (metres and degrees from the rear axle centre, x forward, y left, z up)."""
    robot = ET.Element('robot', name='heven_ioniq5')
    ET.SubElement(robot, 'link', name='base_link')

    def fixed(parent, child, xyz, rpy):
        ET.SubElement(robot, 'link', name=child)
        joint = ET.SubElement(robot, 'joint', name=child + '_joint', type='fixed')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'origin', xyz=' '.join(str(float(v)) for v in xyz), rpy=' '.join(str(float(v)) for v in rpy))

    fixed('base_link', 'rear_axle_link', (0, 0, mounts['axle_height_m']), (0, 0, 0))
    sensors = [('gps_link', mounts['gps']), ('imu_link', mounts['imu'])]
    if mounts['lidar'].get('enabled', True):
        sensors.append(('lidar_link', mounts['lidar']))
    for name, sensor in (mounts.get('cameras') or {}).items():
        if sensor.get('enabled', True):
            sensors.append((f'camera_{name}_link', sensor))
    for frame, s in sensors:
        fixed('rear_axle_link', frame, [s[k] for k in ('x', 'y', 'z')], [math.radians(s[k]) for k in ('roll', 'pitch', 'yaw')])
        if frame.startswith('camera_'):                 # optical frame: z forward, x right, y down
            fixed(frame, frame.replace('_link', '_optical_frame'), (0, 0, 0), (-math.pi / 2, 0, -math.pi / 2))
    return ET.tostring(robot, encoding='unicode')


def generate_launch_description():
    share = get_package_share_directory(PKG)
    cfg = os.path.join(share, 'config', 'bringup.yaml')
    with open(os.path.join(share, 'config', 'sensor_mounts.yaml'), encoding='utf-8') as f:
        description = robot_description(yaml.safe_load(f))
    arg = LaunchConfiguration
    sim_time = {'use_sim_time': arg('use_sim_time')}
    files = {'course_json': arg('course_json'), 'route_csv': arg('route_csv')}
    perception = IfCondition(arg('perception'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='MIT: the simulator /clock'),
        DeclareLaunchArgument('course_json', default_value='', description='empty: share/hk_mit/course/course.json'),
        DeclareLaunchArgument('route_csv', default_value='', description='empty: share/hk_mit/course/route.csv'),
        DeclareLaunchArgument('target_speed', default_value='5.0', description='m/s'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('perception', default_value='false', description='true: also start the perception nodes'),
        DeclareLaunchArgument('sensor_tf', default_value='true', description='publish the sensor TF from config/sensor_mounts.yaml'),
        Node(package=PKG, executable='localization', output='screen', parameters=[cfg, sim_time, files]),
        Node(package=PKG, executable='planner', output='screen', parameters=[cfg, sim_time, files]),
        Node(package=PKG, executable='control', output='screen',
             parameters=[cfg, sim_time, {'target_speed': arg('target_speed')}]),
        Node(package=PKG, executable='lidar_perception', output='screen', condition=perception, parameters=[sim_time]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='hk_mit_robot_state_publisher',
             output='screen', condition=IfCondition(arg('sensor_tf')),
             parameters=[sim_time, {'robot_description': description}]),
        Node(package='rviz2', executable='rviz2', output='log', condition=IfCondition(arg('rviz')), parameters=[sim_time],
             arguments=['-d', os.path.join(share, 'rviz', PKG + '.rviz')]),
    ])
