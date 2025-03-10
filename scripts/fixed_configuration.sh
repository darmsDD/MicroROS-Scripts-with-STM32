#! /bin/bash

# =====================================================================
# Script Name:     fixed_configurations.sh
# Description:     File with configurations you should not change.
# Author:          Ivan Diniz Dobbin (ivandinizdobbin2@gmail.com)
# Date Created:    26/01/2025
# Last Modified:   26/01/2025
# Version:         1.0.0
# =====================================================================
# Notes:
#   - This script was only tested in a ubuntu 24.04.01 LTS with bash.
#
# =====================================================================

micro_utils_folder_name="micro_ros_stm32cubemx_utils" # Name of the micro ros utils folder
stm32_project_path="$stm32_workspace_name/$stm32_project_name" # The path into your stm32 project

micro_utils_folder_path_to_inside="$stm32_project_path/$micro_utils_folder_name" # The path into micro_utils folder
micro_utils_path_to_extra_resources="$micro_utils_folder_path_to_inside/extra_sources" # The path into micro_utils extra sources
micro_ros_agent_path_to_inside="$stm32_project_path/$microROS_agent_folder_name" # The path into micro ros agent folder.

