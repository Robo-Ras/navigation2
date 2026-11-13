# Copyright (c) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# [Humble port] Upstream defaults params_file to nav2_bringup/params/nav2_params.yaml.
# nav2_bringup is not installed here, so params_file is optional.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    scan_frame_id = LaunchConfiguration('scan_frame_id')
    has_params = PythonExpression(["'", params_file, "' != ''"])

    common = dict(
        package='nav2_loopback_sim',
        executable='loopback_simulator',
        name='loopback_simulator',
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value='',
                              description='Optional ROS 2 parameters file'),
        DeclareLaunchArgument('scan_frame_id', default_value='base_scan'),
        Node(**common, parameters=[params_file, {'scan_frame_id': scan_frame_id}],
             condition=IfCondition(has_params)),
        Node(**common, parameters=[{'scan_frame_id': scan_frame_id}],
             condition=UnlessCondition(has_params)),
    ])
