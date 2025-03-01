#! /bin/bash

# =====================================================================
# Script Name:     style.sh
# Description:     File with functions related to stylization.
# Author:          Ivan Diniz Dobbin (ivandinizdobbin2@gmail.com)
# Date Created:    26/01/2025
# Last Modified:   26/01/2025
# Version:         1.0.0
# =====================================================================
# Notes:
#   - This script was only tested in a ubuntu 24.04.01 LTS with bash.
#
# =====================================================================


#Colors for text
BBlack='\033[1;30m'       # Black
BYellow='\033[1;33m'      # Yellow
BPurple='\033[1;35m'      # Purple
BRed='\033[1;31m'         # Red
Color_Off='\033[0m'       # No color
BGreen='\033[1;32m'       # Green
BCyan='\033[1;36m'        # Cyan
On_Black='\033[40m'       # Black background
stage_over_char="="

#====================================================================================
#   Description:    Prints a sentence with the standard color, white.
#   Arguments:      1- The sentence to be printed.
#                   2- Additional flag to echo.
#====================================================================================
Style_NormalSentence(){
    echo -e $2 ${Color_Off}$1
}


#====================================================================================
#   Description:    Prints a sentence in a specific color, each color has a meaning.
#   Arguments:      
#                   1- Type of the sentence, with the following options:
#                       1.1- error (prints sentence in red)
#                       1.2- warning (prints sentence in purple)                        
#                       1.3- important (prints sentence in cyan)
#
#                   2- The sentence to be printed.
#                   3- Additional flag to echo.
#
#   Examples: 1- Style_Sentence error "Testing an error"
#               Prints the message in red.
#
#            2- Style_Sentence warning "Testing a warning\n\n" -n
#               Prints the message in yellow with 2 newlines (\n) after.
#====================================================================================
Style_Sentence(){
    shopt -s nocasematch  # Enable case-insensitive matching
    case $1 in
    error)
        ChoosenColor=$BRed
        ;;

    warning)
        ChoosenColor=$BPurple
        ;;

    important)
        ChoosenColor=$BCyan
        ;;

    success)
        ChoosenColor=$BGreen
        ;;

    *)
        ChoosenColor=$Color_Off
        ;;
    esac
    echo -e $3 ${ChoosenColor}$2${Color_Off}
    shopt -u nocasematch  # Disable case-insensitive matching
}

#====================================================================================
#   Description:    Indicates with echo the end of a stage.
#   Arguments:      None
#====================================================================================
Style_StageOver (){
    # Char used to indicate that a stage is over. tput cols returns the number of columns
    # of the terminal window
    repeated_char=$(printf "$stage_over_char%.0s" $(seq 1 $(tput cols))) 
	echo  -e "${On_Black}$repeated_char${Color_Off}"
	echo -e "${On_Black}$repeated_char${Color_Off}\n"
}


#====================================================================================
#   Description:   Indicates with echo the beginning of a stage.
#
#   Arguments:     1- Stage sentence shown to user. Example: 
#                       "0- Defining the workspace and the project"
#                  2- Additional flag to echo.
#====================================================================================
Style_StageInit(){
    echo -e $2 ${BYellow}$1"\n"${Color_Off} 
}


#====================================================================================
#   Description:    Indicates with echo the beginning of a substage.
#
#   Arguments:      1- Stage sentence shown to user. 
#                       Example: "1.2- Defining the project"
#                   2- Additional flag to echo.
#====================================================================================
Style_SubStageInit(){
    echo -e $2 "\t"${BYellow}$1"\n"${Color_Off} 
}


#====================================================================================
#   Description:   Prints a char n times.
#
#   Arguments:     1- Char to be printed.
#                  2- Number of times the char will be printed.
#====================================================================================
Style_RepeatChar(){
  char=$1
  count=$2
  printf "$char%.0s" $(seq 1 $count)
}


#====================================================================================
#   Description:    Formats a multiline sentence to be used in a sed command.
#   Arguments:      1- Multiline sentence.
#====================================================================================
Style_FormatMultilineVarForSed(){
    sed_multiline_array="$@"
    formatted_multine=$(echo "$sed_multiline_array" | sed "s/ /\\\\\n/g")
}





