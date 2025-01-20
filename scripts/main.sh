#! /bin/bash



# Initial setup to provide access to necessary functions.
. ./your_configuration.sh # File with configurations you should change 
. ./fixed_configuration.sh # File with configurations you should not change
. ./style.sh # File with stylization
. ./base_functions.sh # File with base functions, such as finding directiories, checking for errors in functions, etc.
. ./microROS_functions.sh # File with functions related to microROS, such as cloning the correct repository, starting the agent, etc.
. ./stm32Cube_functions.sh # File with functions related to STM32, such as startting STM32CubeIDE and building the project 


trap BaseFunctions_TerminateProgram SIGINT

cd ..


Style_StageInit "0- Searching for the directory $micro_utils_folder_name"
BaseFunctions_ExecuteFunctionAndCheckError BaseFunctions_FindFolder $micro_utils_folder_path_to_inside
Style_StageOver

Style_StageInit "1- Searching for the directory $microROS_agent_folder_name"
BaseFunctions_ExecuteFunctionAndCheckError BaseFunctions_FindFolder $microROS_agent_folder_name
Style_StageOver

Style_StageInit "2- Building microRos agent and installing dependencies"
BaseFunctions_ExecuteFunctionAndCheckError MicroRos_InitialSetup
Style_StageOver


Style_StageInit "3- Creating microRos agent"
BaseFunctions_ExecuteFunctionAndCheckError MicrosRos_AgentSetup
Style_StageOver

Style_StageInit "4- Building docker"
BaseFunctions_ExecuteFunctionAndCheckError STM32Cube_PrebuildDocker
Style_StageOver

STM32Cube_AlterProjectProperties

STM32Cube_AlterIOCProperties

# Style_StageInit "4- Starting STM32CubeIDE"
# ExecuteFunctionAndCheckError SetupStm32CubeIde


# sub_Style_StageInit "4.1- Building stm32 project"
# ExecuteFunctionAndCheckError BuildStm32CubeProject  


# sub_Style_StageInit "4.2- Flashing code to board"
# ExecuteFunctionAndCheckError FlashCodeToBoard  





# # #gnome-terminal -- bash -c 'tmux send-keys -t $session_name:$window_name.0 "echo "arroz"" Enter'
BaseFunctions_KillProcessTree $$ # kill the parent process and its descendents
wait # ## Espera os processos finalizarem
