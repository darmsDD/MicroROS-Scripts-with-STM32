#! /bin/bash

# =====================================================================
# Script Name:     microROS_functions.sh
# Description:     File with functions related to microROS, such as 
#                  cloning the correct repository, starting the agent 
#                  and others.
#                  
# Author:          Ivan Diniz Dobbin (ivandinizdobbin2@gmail.com)
# Date Created:    26/01/2025
# Last Modified:   26/01/2025
# Version:         1.0.0
# =====================================================================
# Notes:
#   - This script was only tested in a ubuntu 24.04.01 LTS with bash.
#
# =====================================================================

stm32cubeideExec=0

: '
    Purpose:    Does the initial setup of microROS, such as finding the distro and 
                building packages and dependecies.
    Arguments:  None.
'
MicroRos_InitialSetup() {
    MicroRos_FindDistro 
    cd $micro_ros_agent_path_to_inside
    rosdep install --from-path src --ignore-src -y --rosdistro $my_ros_distro
    if [ -d "src" ] && [ -d "build" ] && [ -d "install" ] && [ -d "log" ]; then
        Style_PurpleWord "There is a current build. Skipping this step."
    else
        colcon build
        colcon test
    fi
    source install/local_setup.bash
    cd - &>/dev/null
}

: '
    Purpose: Execute scripts necessary for microROS to function.
    Arguments: None
'
MicrosRos_AgentSetup(){
    cd $micro_ros_agent_path_to_inside
    if [ -f src/ros2.repos ]
    then
        Style_PurpleWord "The setup was done previously. Skipping this step."
    else
        ros2 run micro_ros_setup create_agent_ws.sh
        ros2 run micro_ros_setup build_agent.sh    
    fi
    cd - &>/dev/null
}

: '
    Purpose:    Runs the microROS agent. Allowing the user to visualize the processes in greater detail.
                For example: observing who has subscribed.
    Arguments:  None.
'
MicrosRos_AgentRun(){
    cd $micro_ros_agent_path_to_inside
    source install/local_setup.bash 
    ros2 run micro_ros_agent micro_ros_agent serial --dev $(ls /dev/serial/by-id/*) -b 115200
    cd - &>/dev/null
}

: '
    Purpose:    Check if the ros2 topic is avaible, if it is, echo the messages in the terminal. If not, warn the user and try again.
    Arguments:  1: 1 - Name of topic (topic_name).
'
MicroRos_IsTopicAvaible(){
    export ROS_DOMAIN_ID=$my_ros_domain_id
    local topic_name=$1
    Style_PurpleWord "Flash the programm to the microcontroller or press the reset button if the code was already flashed to the board."
    ros2 topic echo $topic_name
    while [[ $? -ne 0 ]]; do
        Style_RedWord "Topic unavaible, flash the programm to the microcontroller. I will wait 5 seconds and try again."
        sleep 5
        ros2 topic echo $topic_name
    done
}

: '
    Not implemented yet
    Purpose:    Creates a brigde between gazebo and ros2 messages.
    Arguments:  1:  1- An array with N elements in which each element has size 3: Topic name, Ros topic message type, Gazebo topic message type. 
'
# MicroRos_RosBridge(){
#     export ROS_DOMAIN_ID=$my_ros_domain_id
#     bridge_actuator=$topic_velocity_name@$actuator_ros_message_type@$actuator_gazebo_msg_type 
#     bridge_imu=$topic_imu_name@$imu_ros_message_type@$imu_gazebo_msg_type
#     ros2 run ros_gz_bridge parameter_bridge $bridge_actuator $bridge_imu &
# }


: '
    Purpose: Create a option menu and make the user choose one.
    Arguments: The options in an array format.
'
MicroRos_ChooseAnOption(){
    options_array=($@)
    for ((i = 0; i < $number_of_options; i++)); do
        echo "($((i))): ${options_array[$i]}"
    done

    while true; 
    do
        read -p "Select an option  (0-$((number_of_options -1))): " choosen_option
        # Check if input is a number and then checks if it is inside of range 1 to number_of_options
        if [[ $choosen_option =~ ^[0-9]+$ ]] && (($choosen_option >= 0  && $choosen_option<$number_of_options )); then
            break;
        else
            echo "Invalid choice, try again."
        fi
    done


}

: ' 
    Purpose: Finding the user ros2 distro
    Arguments: None
'
MicroRos_FindDistro(){
    # Use find to locate directories named 'ros'
    Style_YellowWord "Finding Ros Distro"
    if [ -n $ROS_DISTRO ]; then
        echo "The variable ROS_DISTRO is not empty."
        my_ros_distro=$ROS_DISTRO
        return 
    fi

    # If you have multiple ros distros, and want a specific one, you should alter the ros2_distros, because it automatically chooses the first one
    # found based on the list order.
    # Example: ros2_distros=("humble" "kilted"), will always choose humble if it is installed.
    ros2_distros=("rolling" "kilted" "jazzy" "iron" "humble" "galactic" "foxy" "eloquent" "dashing" "crystal" "bouncy" "ardent")

    # 1: search in the most common place, /opt/ros/
    for ros2_distro in ${ros2_distros[@]};
    do
        Style_NormalWorld "Searching for $ros2_distro"
        my_ros_distro=$(find /opt/ros/ -type d -name "$ros2_distro" 2>/dev/null)
        if [[ -n $my_ros_distro ]]; then 
            break 
        fi
    done 
    # 2: If no distro was found in /opt/ros, search everywhere.
    if [[ -z $my_ros_distro ]]; then
        for ros2_distro in ${ros2_distros[@]};
        do
            Style_NormalWorld "Searching for $ros2_distro"
            my_ros_distro=$(find / -type d -path "*/ros/$ros2_distro" 2>/dev/null)
            if [[ -n $my_ros_distro ]]; then 
                break 
            fi
        done
    fi
    # 3: If no distro was found, warn the user.
    if [[ -z $my_ros_distro ]]; then
        Style_RedWord "No Ros2 distro was found. Install a ros2 distro. A list of Ros2 distros can be found bellow:\n"
        for ros2_distro in ${ros2_distros[@]};
        do
           Style_RedWord $ros2_distro
        done
        return ;
    fi

    # 4: Get the distro name from path
    my_ros_distro=$(basename $my_ros_distro)     
}
