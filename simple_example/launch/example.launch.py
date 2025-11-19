import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    rviz_file = '/home/developer/ros2_ws/src/simple_example/xd.rviz'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r robot_with_track.sdf'}.items(),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    camera = Node(
        package='simple_example',
        namespace='simple_example',
        executable='camera',
        name='camera'
    )

    control = Node(
        package='simple_example',
        namespace='simple_example',
        executable='control',
        name='control'
    )

    LaserScanController = Node(
        package='simple_example',
        namespace='simple_example',
        executable='LaserScanController',
        name='LaserScanController'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gz_sim,
        bridge,
        rviz,
        camera,
        control,
        # LaserScanController
    ])