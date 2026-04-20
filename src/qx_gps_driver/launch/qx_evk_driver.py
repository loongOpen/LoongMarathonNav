import os
import yaml
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG = "qx_evk_driver"


def get_cfg_path(cfg):
    return os.path.join(get_package_share_directory(PKG), "config", cfg)


def load_yaml(p):
    with open(p, 'r') as f:
        return yaml.safe_load(f)


def get_params(cfg):
    return load_yaml(get_cfg_path(cfg))


def generate_launch_description():
    # 返回 Launch 描述
    return LaunchDescription([
        Node(
            package='qx_evk_driver',
            executable='qx_evk_driver_node',
            name='qx_data_sender',
            output='screen',
            parameters=[
                get_params("qx_evk_cfg.yaml")  # 从 YAML 文件加载参数
            ]
        )
    ])
