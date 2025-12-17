#!/usr/bin/env python3
"""
system.launch.py - Complete crane automation system launcher
Uses single unified config file (system.yaml)
"""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 패키지 경로
    pkg_name = 'hyundai_steel_crane_automation_system'
    pkg_path = get_package_share_directory(pkg_name)
    
    # 통합 Config 파일 경로
    config = os.path.join(pkg_path, 'config', 'config.yaml')
    
    return LaunchDescription([
        # Socket Bridge Node (PLC 통신)
        Node(
            package=pkg_name,
            executable='socket_node.py',
            name='socket_bridge_node',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        ),
        
        # Inference Node (SAM2 기반 측정)
        Node(
            package=pkg_name,
            executable='inference_node.py',
            name='inference_node',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        )
    ])
