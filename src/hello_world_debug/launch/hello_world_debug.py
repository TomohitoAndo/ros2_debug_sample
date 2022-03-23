# Copyright 2021 Tier IV, Inc. All rights reserved.
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

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():

    hello_world_debug_component = ComposableNode(
        package="hello_world_debug",
        plugin="hello_world_debug::TalkerComponent",
        name="hello_world_debug",
    )

    container = ComposableNodeContainer(
        name="hello_world_debug_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            hello_world_debug_component,
        ],

        # use debugger
        # prefix=["xterm -e gdb -ex run --args"],
        prefix=["xterm -e gdb --args"],
    )

    return launch.LaunchDescription(
        [
            container
        ]
    )
