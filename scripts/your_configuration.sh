#! /bin/bash


microROS_agent_folder_name="microROS_agentFolder" # Name of the microROS agent folder, you can choose whatever you want.
my_ros_domain_id=25 # The ros domain id


session_name=micro_ros # Tmux session name
window_name=window1 # Tmux window name
full_path="$session_name:$window_name" # Tmux full path used to send commands to different panes.