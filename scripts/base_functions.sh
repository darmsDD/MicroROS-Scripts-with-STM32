#! /bin/bash


# Responsible for checking that the micro_ros_stm32cubemx_utils folder exists, as it is essential.
# 1- If the folder doesn't exist, clone the repository.
# 2- Rewrite the extra_packages.repos file, adding the packages needed for the project.
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

BaseFunctions_CreateUtilsFolder(){
    git -C $stm32_project_path clone https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git
    cp scripts/extra_packages.repos "$micro_utils_folder_path_to_inside/microros_static_library_ide/library_generation/extra_packages/extra_packages.repos"
    cp scripts/extra_packages.repos "$micro_utils_folder_path_to_inside/microros_static_library/library_generation/extra_packages/extra_packages.repos"
    cd $micro_utils_path_to_extra_resources
    cp ./microros_time.c ./microros_allocators.c ./custom_memory_manager.c ./microros_transports/dma_transport.c ../../Core/Src
    cd - &>/dev/null
}

BaseFunctions_CreateMicroRosAgentFolder(){
    mkdir -p $micro_ros_agent_path_to_inside
    git -C $micro_ros_agent_path_to_inside clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup 
}


# Asks if you want to execute the steps previously declared.
# If so, it executes the function passed as the first argument.
# If not, it exist the program.
# Argument 1: Function to be executed
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


# Runs the function and checks if there were any errors. In the event of an error, it executes the cleanup function.
# Argument 1: function to be executed
# Argument 2: cleanup function to be executed in case of error. 
BaseFunctions_ExecuteFunctionAndCheckError(){
    local my_function=$1
    local arguments=(${@:2}) # take the arguments of the my_function
    $my_function $arguments # executes the function passed as argument
    if [[ "$?" = 0 ]]; then
        Style_GreenWord "Success"
    else
        Style_RedWord "Something went wrong. Check the called functions:"
        Style_PurpleWord "$my_function ${FUNCNAME[*]}"
        $terminateProgram # performs the cleanup function in case of failure (removing files and so on that were created during the process).
        exit $?
    fi
}

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

# Responsible for deleting all descendants of a parent process.
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


BaseFunctions_Menu(){
    list=($1)
    type=$2
    local -n var_to_change=$3
    #echo "${list[0]}"
    while true;
        do
        i=0
        for item in ${list[@]};
        do
            Style_NormalWorld "$i:$item"
            i=$((i+1))
        done
        read -p "Choose a(n) $type:" input
        echo $i
        if [[ $input =~ ^[0-9]+$ ]] && ((input>0 && input<$i)); then
            Style_YellowWord "You choose: ${list[$input]}"
            var_to_change=${list[$input]}
            break
        else
            Style_YellowWord "Invalide choice, your choice must be a number between 0 and $i."
        fi
    done 
}



BaseFunctions_SetWorkspaceAndProject(){

    if dpkg-query -W zenity 2>/dev/null | grep -q "zenity"; then
        Style_GreenWord "Zenity is installed."
    else
        Style_YellowWord "Zenity is not installed."
        Style_YellowWord "Installing zenity."
        sudo apt-get install zenity
    fi

    choosen_project=$(zenity --file-selection --directory --title="Select a project" 2>/dev/null)
    stm32_workspace_name=$(dirname $choosen_project)
    stm32_project_name=$(basename $choosen_project)
    . scripts/fixed_configuration.sh # File with configurations you should not change 

}