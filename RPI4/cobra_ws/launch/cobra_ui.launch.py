#!/usr/bin/env python3
# ui_launch.py - RPi4 - LAUNCH SIMPLIFIÉ
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, ExecuteProcess, TimerAction

def generate_launch_description():
    home_dir = os.path.expanduser('~')
    
    # UI Controller Node
    ui_controller_node = Node(
        package='cobra_control1',
        executable='ui_controller1',
        name='cobra_ui_controller1',
        output='screen'
    )
    
    # ROSBridge WebSocket
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0'
        }]
    )
    
    # Serveur Web
    web_server = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['node', os.path.join(home_dir, 'cobra_web', 'server.js')],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        LogInfo(msg='🚀 Démarrage CoBra RPi4 - COMMUNICATION CLAIRE'),
        ui_controller_node,
        rosbridge_node,
        web_server,
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='✅ RPi4 prêt!'),
                LogInfo(msg=' Interface: http://localhost:3000'),
                LogInfo(msg=' ROSBridge: ws://localhost:9090'),
                LogInfo(msg=' Topic vers RPi5: /rpi4_to_rpi5'),
                LogInfo(msg=' Topic depuis RPi5: /rpi5_to_rpi4')
            ]
        )
    ])
