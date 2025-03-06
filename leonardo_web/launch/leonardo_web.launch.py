from launch_ros.actions import Node

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare 


def generate_launch_description():

    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~
    package_name = 'leonardo_web'
    package_rosbridge = "rosbridge_server"
    package_vision = "leonardo_vision"
    package_camera = "test_package"
    package_web_video_server = "web_video_server"

    #~~~~~~~~~~~~~~~~~~~~~~~~ ARGUMENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    arg_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value= "false",
        description='Use simlation or not'
    )

    config_use_sim = LaunchConfiguration('use_sim')



    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~
    path_rosbridge = PathJoinSubstitution([FindPackageShare(package_rosbridge),"launch","rosbridge_websocket_launch.xml",])
    path_camera = PathJoinSubstitution([FindPackageShare(package_camera),"launch","publisher_camera.launch.py",])

    path_config = PathJoinSubstitution([FindPackageShare(package_name),'config','web.yaml'])
    


    #~~~~~~~~~~~~~~~~~~~~~~~~~ LAUNCH ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    rosbridge_websocket = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(path_rosbridge)
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path_camera)
    )

    #~~~~~~~~~~~~~~~~~~~~~~~~~ NODES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    web_video_server = Node(
        package=package_web_video_server,
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[path_config, {"use_sim_time": config_use_sim}]
    )

    return LaunchDescription([
        arg_use_sim,
        rosbridge_websocket,
        web_video_server,
        camera
    ])