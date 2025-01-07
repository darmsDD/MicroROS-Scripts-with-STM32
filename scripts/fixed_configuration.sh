#! /bin/bash

micro_utils_folder_name="micro_ros_stm32cubemx_utils"
micro_utils_folder_path="$stm32_workspace_name/$stm32_project_name"
micro_utils_folder_path_to_inside="$micro_utils_folder_path/$micro_utils_folder_name"
micro_utils_path_to_extra_resources="$micro_utils_folder_path_to_inside/extra_sources"


micro_ros_agent_folder_clone_command="git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup"
micro_utils_folder_clone_command="git -C $micro_utils_path clone https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git"
extra_package_command="cp scripts/extra_packages.repos $micro_utils_folder_path_to_inside/microros_static_library_ide/library_generation/extra_packages/extra_packages.repos"
