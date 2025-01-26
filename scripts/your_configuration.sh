#! /bin/bash

# =====================================================================
# Script Name:     your_configuration.sh
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

microROS_agent_folder_name="microROS_agentFolder" # Name of the microROS agent folder, you can choose whatever you want.
my_ros_domain_id=25 # The ros domain id


session_name=micro_ros # Tmux session name
window_name=window1 # Tmux window name
full_path="$session_name:$window_name" # Tmux full path used to send commands to different panes.