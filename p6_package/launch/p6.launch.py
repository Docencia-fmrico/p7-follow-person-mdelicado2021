# Copyright 2024 Mario Delicado Díaz
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

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    Yolo_cmd = Node(
        package='camera',
        executable='yolo_detection',
        output='screen',
        parameters=[],
        remappings=[
            ('input_detection', '/yolo/detections'),
            ('output_detection_2d', 'detection_2d')
        ]
    )

    convert_2d_3d = Node(
        package='camera',
        executable='detection_2d_to_3d_depth',
        output='screen',
        parameters=[],
        remappings=[
            ('input_depth', '/camera/depth/image_raw')
            ('input_detection_2d', 'detection_2d')
            ('camera_info', '/camera/camera_info')
            ('input_detection_3d', 'detection_3d')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(Yolo_cmd)
    ld.add_action(convert_2d_3d)
    return ld