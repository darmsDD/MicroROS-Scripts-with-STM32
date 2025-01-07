#! /bin/bash



# Initial setup to provide access to necessary functions.
. ./your_configuration.sh # File with configurations you should change 
. ./fixed_configuration.sh # File with configurations you should not change
. ./style.sh # File with stylization
. ./base_functions.sh # File with base functions, such as finding directiories, checking for errors in functions, etc.
. ./microROS_functions.sh # File with functions related to microROS, such as cloning the correct repository, starting the agent, etc.
. ./stm32Cube_functions.sh # File with functions related to STM32, such as startting STM32CubeIDE and building the project 


trap terminateProgram SIGINT

cd ..

stage_init "0- Searching for the directory $micro_utils_folder_name"
ExecuteFunctionAndCheckError FindFolder $micro_utils_folder_path_to_inside
stage_over

stage_init "1- Searching for the directory $microROS_agent_folder_name"
ExecuteFunctionAndCheckError FindFolder $microROS_agent_folder_name
stage_over

stage_init "2- Building microRos agent and installing dependencies"
ExecuteFunctionAndCheckError MicroRosInitialSetup
stage_over

stage_init "3- Creating microRos agent"
ExecuteFunctionAndCheckError MicrosRosAgentSetup
stage_over


stage_init "4- Starting STM32CubeIDE"
ExecuteFunctionAndCheckError SetupStm32CubeIde

sub_stage_init "4.1- Building stm32 project"
ExecuteFunctionAndCheckError BuildStm32CubeProject  
#FlashCodeToBoard
# purple_word "Is your project already on STM32CubeIDE?[Y/n]:" -n
# read input
# if [ $input == "Y" ]; then
#     sub_stage_init "4.1- Building stm32 project"
#     ExecuteFunctionAndCheckError BuildStm32CubeProject   
# fi
# sub_stage_init "4.2- Opening STM32CubeIDE"
# StartStm32CubeIde
# stage_over




# # #gnome-terminal -- bash -c 'tmux send-keys -t $session_name:$window_name.0 "echo "arroz"" Enter'
kill_process_tree $$ # kill the parent process and its descendents
wait # ## Espera os processos finalizarem
