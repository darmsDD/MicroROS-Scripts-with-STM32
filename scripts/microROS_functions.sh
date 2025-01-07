#! /bin/bash

stm32cubeideExec=0
# Faz a build do pacote e instala dependências 
MicroRosInitialSetup() {
    cd $microROS_agent_folder_name
    rosdep install --from-path src --ignore-src -y
    if [ -d "src" ] && [ -d "build" ] && [ -d "install" ] && [ -d "log" ]; then
        purple_word "Build já realizada anteriormente. Pulando esta etapa."
    else
        colcon build
        colcon test
    fi
    source install/local_setup.bash
    cd - &>/dev/null
}


# Executa os scripts necessários para o agente do micro_ros funcionar
MicrosRosAgentSetup(){
    cd $microROS_agent_folder_name
    if [ -f src/ros2.repos ]
    then
        purple_word "Setup já realizado anteriormente. Pulando esta etapa."
    else
        ros2 run micro_ros_setup create_agent_ws.sh
        ros2 run micro_ros_setup build_agent.sh    
    fi
    cd - &>/dev/null
}

# Roda o agente do micro-ros, permitindo observar no terminal com mais detalhes os processos.
# Como por exemplo, ver quem se inscreveu.
MicrosRosAgentRun(){
    cd $microROS_agent_folder_name
    source install/local_setup.bash 
    ros2 run micro_ros_agent micro_ros_agent serial --dev $(ls /dev/serial/by-id/*) -b 115200
    cd - &>/dev/null
}

# Função utilizada para checar se o tópico ROS está disponível, e se não estiver, avisar o usuário o que ele pode fazer.
IsTopicAvaible(){
    export ROS_DOMAIN_ID=$my_ros_domain_id
    local topic_name=$1
    purple_word "Rode o código no microcontrolador ou aperte o botão de reset, caso o código já esteja na placa."
    ros2 topic echo $topic_name
    while [[ $? -ne 0 ]]; do
        red_word "Tópico ainda não está disponível, rode o código do microcontrolador. Esperarei 5 segundos e tentarei novamente."
        sleep 5
        ros2 topic echo $topic_name
    done
}

# Função utilizada para observar no terminal as mensagens sendo enviadas pelo micro-ros, isto é, o Target (Microcontrolador) para o o Host (PC).
RosSubscriber(){
    IsTopicAvaible $topic_velocity_name
}

# Função utilizada para observar no terminal as mensagens sendo enviadas pelo gazebo, isto é, o Host (PC) para o Target (Microcontrolador).
RosPublisher(){
    IsTopicAvaible $topic_imu_name
}

# Função utilizada para transformar as mensagens to tipo ros para gazebo e gazebo para ros. Permitindo a comunicação entre os tópicos.
RosBridge(){
    export ROS_DOMAIN_ID=$my_ros_domain_id
    bridge_actuator=$topic_velocity_name@$actuator_ros_message_type@$actuator_gazebo_msg_type 
    bridge_imu=$topic_imu_name@$imu_ros_message_type@$imu_gazebo_msg_type
    ros2 run ros_gz_bridge parameter_bridge $bridge_actuator $bridge_imu &
}