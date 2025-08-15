# lc29h_gps.launch.py
# 1. build
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select lc29h_gps
#  $ . install/setup.bash
#
# 2. execute
#  $ ros2 launch lc29h_gps lc29h_gps.launch.py
#
# 3. check topic
#  $ ros2 topic info /fix --verbose
#
#import launch
#import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0', description=''),
        DeclareLaunchArgument('frame_id', default_value='gps_link', description=''),
        DeclareLaunchArgument('rate', default_value='1', description=''),
        DeclareLaunchArgument('topicName', default_value='fix', description=''),
        DeclareLaunchArgument('use_dgns', default_value='false', description=''),
        Node(
            package='lc29h_gps',
            executable='lc29h_gps_node',
            output="screen",
            #emulate_tty=True,
            parameters=[
                        {"device": LaunchConfiguration('device') ,
                         "frame_id": LaunchConfiguration('frame_id'),
                         "rate": LaunchConfiguration('rate'),
                         "topicName": LaunchConfiguration('topicName'),
                         "use_dgns": LaunchConfiguration('use_dgns'),
                         }
            ]
        )
    ])

'''
#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="minimal_param_node",
            name="custom_minimal_param_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
'''