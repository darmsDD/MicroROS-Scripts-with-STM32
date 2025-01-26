#! /bin/bash


# =====================================================================
# Script Name:     base_functions.sh
# Description:     Brief description of what the script does.
# Author:          Ivan Diniz Dobbin (ivandinizdobbin2@gmail.com)
# Date Created:    26/01/2025
# Last Modified:   26/01/2025
# Version:         1.0.0
# =====================================================================
# Notes:
#   - This script was only tested in a ubuntu 24.04.01 LTS with bash.
#
# =====================================================================


: '
    Purpose:    Responsible for checking that the micro_ros_stm32cubemx_utils and the micro Ros agent folder exists, as they are essential.
                - If micro_ros_stm32cubemx_utils does not exist, call BaseFunctions_CreateUtilsFolder.
                - If the Micro Ros agent folder ($microROS_agent_folder_name, you can find it in your_configuration.sh) does not exist, call BaseFunctions_CreateMicroRosAgentFolder.
    
    Arguments:  1- Path to inside of the folder. 
                    Example: if the folder is named microROS_agentFolder, the path can be /Desktop/My_project/microROS_agentFolder
'
BaseFunctions_FindFolder() {
    local folder_path_to_inside=$1
    if [ ! -d $folder_path_to_inside ]; then
        Style_PurpleWord "Directory not found." 
        Style_YellowWord "To resolve, the following steps must be taken:\n"
        if [ "$folder_path_to_inside" = "$micro_utils_folder_path_to_inside" ]; then
            local steps=("Clone micro utils repository" "Replace extra_packages.repos with your own" "Copy essential files to $stm32_project_path/Core/Src\n")
            BaseFunctions_AuthorizationInput BaseFunctions_CreateUtilsFolder "${steps[@]}"
        elif [ "$folder_path_to_inside" = "$micro_ros_agent_path_to_inside" ]; then
            local steps=("Create the directory $micro_ros_agent_path_to_inside" "Navigate to $micro_ros_agent_path_to_inside" "Clones micro_ros agent repository")
            BaseFunctions_AuthorizationInput BaseFunctions_CreateMicroRosAgentFolder "${steps[@]}"
        fi
    else
        Style_GreenWord "Directory found."
    fi
}


: '
    Purpose:    1- Clone the repository https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git.
                2- Rewrite the extra_packages.repos file, adding the packages needed for the project.
                3- Add essential files to your STM32 Core/Src folder.
    
    Arguments:  None
'
BaseFunctions_CreateUtilsFolder(){
    git -C $stm32_project_path clone https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git
    cp scripts/extra_packages.repos "$micro_utils_folder_path_to_inside/microros_static_library_ide/library_generation/extra_packages/extra_packages.repos"
    cp scripts/extra_packages.repos "$micro_utils_folder_path_to_inside/microros_static_library/library_generation/extra_packages/extra_packages.repos"
    cd $micro_utils_path_to_extra_resources
    cp ./microros_time.c ./microros_allocators.c ./custom_memory_manager.c ./microros_transports/dma_transport.c ../../Core/Src
    cd - &>/dev/null
}


: '
    Purpose:    Create a folder named $microROS_agent_folder_name (you can find it in the your_configuration.sh) and clone the repository https://github.com/micro-ROS/micro_ros_setup.git. 
    
    Arguments:  None.
'
BaseFunctions_CreateMicroRosAgentFolder(){
    mkdir -p $micro_ros_agent_path_to_inside
    git -C $micro_ros_agent_path_to_inside clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup 
}


: '
    Purpose:    Asks if you want to execute the steps declared.
                If so, it executes the function passed as the first argument.
                If not, it exist the program.
               
    Arguments:  1- Function to be executed
                2- String array with every step to be executed. 
                    Example: local steps=("Create the directory $micro_ros_agent_path_to_inside" "Navigate to $micro_ros_agent_path_to_inside" "Clones micro_ros agent repository")
'
BaseFunctions_AuthorizationInput(){
    local my_function=$1
    local steps=("${@:2}")
    local i=1
    for var in "${steps[@]}"
    do
        Style_NormalWorld "\t$i-$var\n"
        ((i++))
    done
    Style_YellowWord "Do you authorize the execution of the following steps:[Y/n]:" -n
    read input
    if [ $input == "Y" ]; then
        $my_function
    else
        Style_RedWord "Authorization was not granted."
        BaseFunctions_TerminateProgram
    fi
    
}


: '
    Purpose:    Runs the function and checks if there were any errors. In the event of an error, it prints the stack of functions
                called, and then executes the cleanup function.  
    Arguments:  1- Function to be executed
                2- Arguments of the function to be executed. You can also pass 0 arguments.
'
BaseFunctions_ExecuteFunctionAndCheckError(){
    local my_function=$1
    local arguments=(${@:2}) # take the arguments of the my_function
    $my_function $arguments # executes the function passed as argument
    if [[ "$?" = 0 ]]; then
        Style_GreenWord "Success"
    else
        Style_RedWord "Something went wrong. Check the called functions:"
        Style_PurpleWord "$my_function ${FUNCNAME[*]}"
        BaseFunctions_TerminateProgram # performs the cleanup function in case of failure (removing files and so on that were created during the process).
    fi
}


: '
    Purpose:    Terminates the program in case of error or user pressing Ctrl+c.
                It asks the user if it can remove the micro_ros_stm32cubemx_utils and micro Ros agent folder, kills all
                child processes and finalize the program.    
    Arguments:  None.
'
BaseFunctions_TerminateProgram(){
    Style_YellowWord "\nRemoving $microROS_agent_folder_name and $micro_utils_folder_name. Do you authorize?[Y/n]:" -n
    read input
    if [ "$input" == "Y" ]; then
       #ExecuteFunctionAndCheckError $my_function $my_clean_up_function
       rm -rf $micro_utils_folder_path_to_inside
       rm -rf $microROS_agent_folder_name
    else
        Style_RedWord "Authorization was not granted."
    fi

    Style_PurpleWord "Terminating Program"
    BaseFunctions_KillProcessTree $$
    wait
    exit
}


: '
    Purpose:    Kills all descendants of a parent process.    
    Arguments:  1- Parent process pid.
'
BaseFunctions_KillProcessTree() {
    local parent_pid=$1
    # Find all child processes
    local child_pids=$(pgrep -P $parent_pid)
    for child_pid in $child_pids; do
        # Recursively kill child processes
        BaseFunctions_KillProcessTree $child_pid
    done
    # Kill the parent process if it is not the original one
    if [ $parent_pid -ne $$ ]; then
      kill -SIGTERM $parent_pid 2>/dev/null
    fi
}


: '
    Purpose:    Generates a random 10 digit number between 1000000000-9999999999. Currently used by ioc file.
    Arguments:  None.
'
BaseFunctions_GenerateRandomNumber(){
    while true; do
        random_10_digit_number=$(shuf -i 1000000000-9999999999 -n 1)
        # Check if the number exists in the file
        if ! grep -q "$random_10_digit_number" "$XML_FILE"; then
            echo "$random_10_digit_number"
            # If not found, break the loop
            break
        fi
    done

}


: '
    Purpose:    Allows the user to choose a STM32 project to configure.
                It also checks if zenity is installed and asks permission to install if it is not present.
    Arguments:  None.
'
BaseFunctions_SetWorkspaceAndProject(){

    if dpkg-query -W zenity 2>/dev/null | grep -q "zenity"; then
        Style_PurpleWord "Zenity is installed."
    else
        Style_YellowWord "Zenity is not installed."
        install_zenity="sudo apt-get install zenity"
        local steps=("Install zenity")
        BaseFunctions_AuthorizationInput  "$install_zenity"  "${install_zenity}"
    fi

    Style_YellowWord "Choose your project."
    choosen_project=$(zenity --file-selection --directory --title="Select a project" 2>/dev/null)
    stm32_workspace_name=$(dirname $choosen_project)
    stm32_project_name=$(basename $choosen_project)
    . scripts/fixed_configuration.sh # File with configurations you should not change 

}