#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    veh_arg = DeclareLaunchArgument(
        'veh',
        default_value='megaman',
        description='Name of vehicle. ex: megaman'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port for the websocket server'
    )
    
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Address to bind to'
    )
    
    ssl_arg = DeclareLaunchArgument(
        'ssl',
        default_value='false',
        description='Enable SSL'
    )
    
    certfile_arg = DeclareLaunchArgument(
        'certfile',
        default_value='',
        description='SSL certificate file'
    )
    
    keyfile_arg = DeclareLaunchArgument(
        'keyfile',
        default_value='',
        description='SSL key file'
    )
    
    retry_startup_delay_arg = DeclareLaunchArgument(
        'retry_startup_delay',
        default_value='5.0',
        description='Retry startup delay'
    )
    
    fragment_timeout_arg = DeclareLaunchArgument(
        'fragment_timeout',
        default_value='600',
        description='Fragment timeout'
    )
    
    unregister_timeout_arg = DeclareLaunchArgument(
        'unregister_timeout',
        default_value='10.0',
        description='Unregister timeout'
    )
    
    authenticate_arg = DeclareLaunchArgument(
        'authenticate',
        default_value='false',
        description='Enable authentication'
    )
    
    topics_glob_arg = DeclareLaunchArgument(
        'topics_glob',
        default_value='["*"]',
        description='Topics glob pattern'
    )
    
    services_glob_arg = DeclareLaunchArgument(
        'services_glob',
        default_value='["*"]',
        description='Services glob pattern'
    )
    
    params_glob_arg = DeclareLaunchArgument(
        'params_glob',
        default_value='["*"]',
        description='Parameters glob pattern'
    )
    
    bson_only_mode_arg = DeclareLaunchArgument(
        'bson_only_mode',
        default_value='false',
        description='BSON only mode'
    )

    # Create nodes
    rosbridge_websocket_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        namespace=LaunchConfiguration('veh'),
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        parameters=[{
            'authenticate': False,
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'retry_startup_delay': 5.0,
            'fragment_timeout': 600,
            'unregister_timeout': 10.0,
            'bson_only_mode': False,
        }]
    )
    
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        namespace=LaunchConfiguration('veh'),
    )

    return LaunchDescription([
        veh_arg,
        port_arg,
        address_arg,
        ssl_arg,
        certfile_arg,
        keyfile_arg,
        retry_startup_delay_arg,
        fragment_timeout_arg,
        unregister_timeout_arg,
        authenticate_arg,
        topics_glob_arg,
        services_glob_arg,
        params_glob_arg,
        bson_only_mode_arg,
        rosbridge_websocket_node,
        rosapi_node,
    ])
