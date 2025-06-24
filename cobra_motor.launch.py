#!/usr/bin/env python3
# motor_launch.py - RPi5 - LAUNCH 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction

def generate_launch_description():
    
    # Motor Controller Node
    motor_controller_node = Node(
        package='cobra_motor_control_py',
        executable='motor_controller',
        name='cobra_motor_controller',
        output='screen',
        parameters=[{
            'step_size': 5.0,
            'precision': 0.1,
            'workspace_x_min': -100.0,
            'workspace_x_max': 100.0,
            'workspace_y_min': -100.0,
            'workspace_y_max': 100.0,
            'workspace_z_min': 0.0,
            'workspace_z_max': 100.0
        }]
    )
    
    return LaunchDescription([
        LogInfo(msg=' Démarrage CoBra RPi5 - COMMUNICATION CLAIRE'),
        motor_controller_node,
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg=' RPi5 prêt - Contrôleur moteur actif'),
                LogInfo(msg=' Interface CAN: can0'),
                LogInfo(msg=' Moteurs: 5 moteurs configurés'),
                LogInfo(msg=' Topic depuis RPi4: /rpi4_to_rpi5'),
                LogInfo(msg=' Topic vers RPi4: /rpi5_to_rpi4'),
                LogInfo(msg=''),
                LogInfo(msg=' COMMUNICATION ACTIVE:'),
                LogInfo(msg='    Réception: Joysticks, Admin, Système'),
                LogInfo(msg='    Envoi: Position XYZ, CAN, Moteurs')
            ]
        )
    ])
