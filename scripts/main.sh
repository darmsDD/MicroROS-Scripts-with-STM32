#! /bin/bash

# =====================================================================
# Script Name:     main.sh
# Description:     File to be executed.
#                  1- chmod +x main.sh
#                  2- ./main.sh
# Author:          Ivan Diniz Dobbin (ivandinizdobbin2@gmail.com)
# Date Created:    26/01/2025
# Last Modified:   26/01/2025
# Version:         1.0.0
# =====================================================================
# Notes:
#   - This script was only tested in a ubuntu 24.04.01 LTS with bash.
#
# =====================================================================




# Initial setup to provide access to necessary functions.
. ./your_configuration.sh # File with configurations you can change 
. ./style.sh # File with stylization
. ./base_functions.sh # File with base functions, such as finding directories, checking for errors in functions, etc.
. ./microROS_functions.sh # File with functions related to microROS, such as cloning the correct repository, starting the agent, etc.
. ./stm32Cube_functions.sh # File with functions related to STM32, such as altering the .ioc file and the project properties.



branch_name=$(git rev-parse --abbrev-ref HEAD)

#Execute a cleanup function if user presses Ctrl+c
trap BaseFunctions_TerminateProgram SIGINT

cd ..

# Define the STM32 project to be configured
Style_StageInit "0- Defining the workspace and the project"
BaseFunctions_ExecuteFunctionAndCheckError BaseFunctions_SetWorkspaceAndProject

# Checks if the micro_utils folder is present, if not, download it and does other necessary procedures.
Style_StageInit "1- Searching for the directory $micro_utils_folder_name"
BaseFunctions_ExecuteFunctionAndCheckError BaseFunctions_FindFolder $micro_utils_folder_path_to_inside
Style_StageOver

# Checks if the micro_utils folder is present, if not, download it.
Style_StageInit "2- Searching for the directory $microROS_agent_folder_name"
BaseFunctions_ExecuteFunctionAndCheckError BaseFunctions_FindFolder $micro_ros_agent_path_to_inside
Style_StageOver

Style_StageInit "3- Building microRos agent and installing dependencies"
BaseFunctions_ExecuteFunctionAndCheckError MicroRos_InitialSetup
Style_StageOver

Style_StageInit "4- Creating microRos agent"
BaseFunctions_ExecuteFunctionAndCheckError MicrosRos_AgentSetup
Style_StageOver

Style_StageInit "5- Building docker"
BaseFunctions_ExecuteFunctionAndCheckError STM32Cube_PrebuildDocker
Style_StageOver

Style_StageInit "6- Changing project properties"
BaseFunctions_ExecuteFunctionAndCheckError STM32Cube_AlterProjectProperties
Style_StageOver

Style_StageInit "7- Changing .ioc file properties"
BaseFunctions_ExecuteFunctionAndCheckError STM32Cube_AlterIOCProperties
Style_StageOver


# kill the descendents of the process
BaseFunctions_KillProcessTree $$ 
wait # ## Wait for the processes to end.
