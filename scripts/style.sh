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

# Este arquivo é resposnsável pelas estilizações, isto é teste em cores
# Indicar o final de operações e etc.

#Colors for text
BBlack='\033[1;30m'       # Black
BYellow='\033[1;33m'        # Blue
BPurple='\033[1;35m'      # Purple
BRed='\033[1;31m'         # Red
Color_Off='\033[0m'       # No color
BGreen='\033[1;32m'       # Green
BCyan='\033[1;36m'        # Cyan
On_Black='\033[40m'       # Black background
stage_over_char="="

: '
    Purpose:    Prints a sentence with the standard color, white.
    Arguments:  1- The sentence to be printed.
                2- Additional flag to echo.
'
Style_NormalWorld(){
    echo -e $2 ${Color_Off}$1
}


: '
    Purpose:    Prints a sentence in red. Used when something went wrong.
    Arguments:  1- The sentence to be printed.
                2- Additional flag to echo.
'
Style_RedWord () {
    echo -e $2 ${BRed}$1${Color_Off}
}


: '
    Purpose:    Prints a sentence in black.
    Arguments:  1- The sentence to be printed.
                2- Additional flag to echo.
'
Style_BlackWord () {
    echo -e $2 ${BBlack}$1${Color_Off}
}


: '
    Purpose:    Prints a sentence in green. Used when an operation was successfull.
    Arguments:  1- The sentence to be printed.
                2- Additional flag to echo.
'
Style_GreenWord () {
    echo -e $2 ${BGreen}$1${Color_Off}
}


: '
    Purpose:    Prints a sentence in Yellow. Used as an alternative for Style_PurpleWord to print messages.
    Arguments:  1- The sentence to be printed.
                2- Additional flag to echo.
'
Style_YellowWord () {
    echo -e $2 ${BYellow}$1${Color_Off}
}


: '
    Purpose:    Prints a sentence in purple. Used as an alternative for Style_YellowWord to print messages. 
    Arguments:  1- The sentence to be printed.
                2- Additional flag to echo.
'
Style_PurpleWord () {
    echo -e $2 ${BPurple}$1${Color_Off}
}

: '
    Purpose:    Indicates with echo the end of a stage.
    Arguments:  None
'
Style_StageOver (){
	echo  -e "${On_Black}$repeated_char${Color_Off}"
	echo -e "${On_Black}$repeated_char${Color_Off}\n"
}


: '
    Purpose:    Indicates with echo the beginning of a stage.
    Arguments:  1- Stage sentence shown to user. Example: "0- Defining the workspace and the project"
                2- Additional flag to echo.
'
Style_StageInit(){
    echo -e $2 ${BYellow}$1"\n"${Color_Off} 
}


: '
    Purpose:    Indicates with echo the beginning of a substage.
    Arguments:  1- Stage sentence shown to user. Example: "1.2- Defining the project"
                2- Additional flag to echo.
'
Style_SubStageInit(){
    echo -e $2 "\t"${BYellow}$1"\n"${Color_Off} 
}


: '
    Purpose:    Prints a char n times.
    Arguments:  1- Char to be printed."
                2- Number of times the char will be printed.
'
Style_RepeatChar(){
  char=$1
  count=$2
  printf "$char%.0s" $(seq 1 $count)
}


: '
    Purpose:    Formats a multiline sentence to be used in a sed command.
    Arguments:  1- Multiline sentence.
'
Style_FormatMultilineVarForSed(){
    sed_multiline_array="$@"
    formatted_multine=$(echo "$sed_multiline_array" | sed "s/ /\\\\\n/g")
}



repeated_char=$(printf "$stage_over_char%.0s" $(seq 1 150)) # Char used to indicate that a stage is over.

