# Copyright 2024 pradyum
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

    dual_laser_merger_node = ComposableNodeContainer(
        name='demo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dual_laser_merger',
                plugin='merger_node::MergerNode',
                name='dual_laser_merger',
                parameters=[
                    {'laser_1_topic': '/sick_lidar0/scan'},
                    {'laser_2_topic': '/sick_lidar1/scan'},
                    {'merged_scan_topic': '/scan'},
                    {'target_frame': 'base_footprint'},
                    {'laser_1_x_offset': 0.0},
                    {'laser_1_y_offset': 0.0},
                    {'laser_1_yaw_offset': 0.0},
                    {'laser_2_x_offset': -0.04},
                    {'laser_2_y_offset': 0.0},
                    {'laser_2_yaw_offset': 0.0},
                    {'tolerance': 0.05},  # AMCL için daha yüksek tolerans
                    {'queue_size': 10},  # Daha büyük queue boyutu
                    {'angle_increment': 0.00436332},  # ~0.25 derece (AMCL için optimize)
                    {'scan_time': 0.1},  # Standart scan süresi
                    {'range_min': 0.1},  # AMCL için minimum range (çok yakın ölçümler filtrelenir)
                    {'range_max': 25.0},  # AMCL laser_max_range ile uyumlu
                    {'min_height': -0.3},  # Zemin seviyesine yakın
                    {'max_height': 0.5},  # Robotun yüksekliğine uygun
                    {'angle_min': -3.141592654},  # Tam 360 derece tarama
                    {'angle_max': 3.141592654},
                    {'inf_epsilon': 1.0},
                    {'use_inf': False},  # AMCL için false olmalı
                    {'allowed_radius': 0.5},  # Robot yarıçapına uygun
                    {'enable_shadow_filter': True},  # Gölge filtresi aktif
                    {'enable_average_filter': True},  # Ortalama filtresi aktif
                    ],
            )
        ],
        output='screen',
    )

    ld.add_action(dual_laser_merger_node)

    return ld
