#! /bin/bash

stm32cubeideExec=0
# Faz a build do pacote e instala dependências 
MicroRos_InitialSetup() {
    MicroRos_FindDistro 
    cd $microROS_agent_folder_name
    rosdep install --from-path src --ignore-src -y
    if [ -d "src" ] && [ -d "build" ] && [ -d "install" ] && [ -d "log" ]; then
        Style_PurpleWord "There is a current build. Skipping this step."
    else
        colcon build
        colcon test
    fi
    source install/local_setup.bash
    cd - &>/dev/null
}

# Executa os scripts necessários para o agente do micro_ros funcionar
MicrosRos_AgentSetup(){
    cd $microROS_agent_folder_name
    if [ -f src/ros2.repos ]
    then
        Style_PurpleWord "The setup was done previously. Skipping this step."
    else
        ros2 run micro_ros_setup create_agent_ws.sh
        ros2 run micro_ros_setup build_agent.sh    
    fi
    cd - &>/dev/null
}

# Roda o agente do micro-ros, permitindo observar no terminal com mais detalhes os processos.
# Como por exemplo, ver quem se inscreveu.
MicrosRos_AgentRun(){
    cd $microROS_agent_folder_name
    source install/local_setup.bash 
    ros2 run micro_ros_agent micro_ros_agent serial --dev $(ls /dev/serial/by-id/*) -b 115200
    cd - &>/dev/null
}

# Função utilizada para checar se o tópico ROS está disponível, e se não estiver, avisar o usuário o que ele pode fazer.
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

# Função utilizada para observar no terminal as mensagens sendo enviadas pelo micro-ros, isto é, o Target (Microcontrolador) para o o Host (PC).
MicroRos_RosSubscriber(){
    IsTopicAvaible $topic_velocity_name
}

# Função utilizada para observar no terminal as mensagens sendo enviadas pelo gazebo, isto é, o Host (PC) para o Target (Microcontrolador).
MicroRos_RosPublisher(){
    IsTopicAvaible $topic_imu_name
}

# Função utilizada para transformar as mensagens to tipo ros para gazebo e gazebo para ros. Permitindo a comunicação entre os tópicos.
MicroRos_RosBridge(){
    export ROS_DOMAIN_ID=$my_ros_domain_id
    bridge_actuator=$topic_velocity_name@$actuator_ros_message_type@$actuator_gazebo_msg_type 
    bridge_imu=$topic_imu_name@$imu_ros_message_type@$imu_gazebo_msg_type
    ros2 run ros_gz_bridge parameter_bridge $bridge_actuator $bridge_imu &
}



# # Function used to created a menu and make the user choose
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

MicroRos_FindDistro(){
    # Use find to locate directories named 'ros'
    Style_YellowWord "Finding Ros Directory"
    ros_directories=$(find / -type d -name "ros" 2>/dev/null)

    # Check if any directories were found
    if [[ -z $ros_directories ]]; then
        echo "No directories named 'ros' found."
        BaseFunctions_TerminateProgram
    fi

    # Convert string to array
    ros_directories_array=($ros_directories)
    # echo ${ros_directories_array[0]}
    # echo ${ros_directories_array[1]}
    #Size of array ${#ros_directories_array[@]}
    number_of_options=${#ros_directories_array[@]}
    MicroRos_ChooseAnOption "${ros_directories_array[@]}"
    choosen_ros_directory=${ros_directories_array[$choosen_option]}
    
    Style_YellowWord "Finding Ros Distro"
    distro_options=($(ls $choosen_ros_directory)) # convert ls output to array
    
    if [[ -z $distro_options ]]; then
        echo "No distros found."
        BaseFunctions_TerminateProgram
    fi

    number_of_options=${#distro_options[@]} 
    MicroRos_ChooseAnOption "${distro_options[@]}"
    ROS_DISTRO=${distro_options[$choosen_option]}
    
}
