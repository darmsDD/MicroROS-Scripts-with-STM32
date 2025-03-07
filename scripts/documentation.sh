#! /bin/bash


#sudo apt install graphviz

#!/bin/bash


# a=("MicroRos_InitialSetup" "MicroRos_FindDistro" "MicroRos_HUAHUAHUAHUHAU")
# i=0
# graph=
# array_size=${#a[@]}
# for (( i=0; i<array_size; i++ ));
# do
#     graph+="${a[i]}"
#     if (( i + 1 < array_size )); then
#         graph+="->"
#     fi
#     echo ${a[i]}
# done
list_of_functions=()

# ================================================================================================================================
#    Description:   Finds every functions declared in a bash file, with the following structure FunctionName () {
#                   Part 1: ignores any commented lines
#                   
#                   Part 2: 1. -P: Perl-compatible regular expressions 
#                           2. -z : Treat input and output data as sequences of lines, each  terminated  by  a  zero
#                                   byte  (the  ASCII  NUL  character)  instead of a newline.
#                           3. -o : Print only the matched (non-empty) parts of a matching line. Because I am using -z,
#                                   the whole file is considered a single line, so after the first match, all the lines would
#                                   be printed without -o option.
#                           4. \w+: matches one or more single word characters (Letters (a-z, A-Z), Digits (0-9), Underscore (_)) 
#                           5. (?=\s*\(\)\s*{): Is a lookahed, that checks if a word if followed by (){, even if there is multiple
#                                   spaces or lines between the function name, () and {. The lookahead is usefull because we only 
#                                   get the function name, without needing to remove () { afterwards.
#
#                   Part 3: replaces \0 (the  ASCII  NUL  character) with \n for better visualization;
#
#    Arguments:     None.
# ================================================================================================================================

Documentation_ListAllFunctionsInFile(){
    name_of_file=$1
    list_of_functions+=($(grep -v "^\s*#" $name_of_file | grep -Pzo '\w+(?=\s*\(\)\s*{)' | tr '\0' '\n'))
}

Documentation_ListAllFunctionsCalls(){
    name_of_function=$1
    called_functions=($(grep -v "^\s*#" $name_of_function | grep -Pzo '\w+(?!\s*\(\)\s*{)' | tr '\0' '\n'))
}




temp_list_of_files=$(ls *.sh)
list_of_files=($temp_list_of_files) # transform in array
number_of_files=${#list_of_files[@]}
for (( i=0; i<number_of_files; i++ ));
do
    echo -e "${list_of_files[i]} \n"
    Documentation_ListAllFunctionsInFile "${list_of_files[i]}"
    # echo -e "====================================================================================="
    # echo -e "=====================================================================================\n\n"
done

echo "${list_of_functions[@]}"
# number_of_functions=${#list_of_functions[@]}

# for ((i=0; i<number_of_functions; i++ ));
# do
#     for ((j=0; j<number_of_files; j++ ));
#     do
#         Documentation_ListAllFunctionsCalls "${list_of_functions[i]}"
#     done
# done


# grep -v "^\s*#" *.sh | grep -n -Pzo '\w+(?=\s*\(\)\s*{)' | tr '\0' '\n' #| awk -F: '{print "Function:", $3, "in file:", $1, "at line:", $2}'


#declare -f teste
# echo "digraph G { rankdir=LR; $graph}" | dot -Tpng -o graph.png
# echo "Graph saved as graph.png"

awk ' BEGIN {count=0;}   
    function identify_call_or_function_declaration(initial_number) {
            for (i=initial_number; i<=NF; i++) {
                current_word=$i
                if (current_word == "(" && count == 0){
                    count=1
                } else if (current_word == ")" && count == 1){
                    #Declaração de função
                    print initial_line_number, "Declaração"
                    next
                }
                else{
                    #Chamada de função 
                    print initial_line_number, "Chamada"
                    next
                }
            }
            
        }

    /^\s*\<apple\>/ {
        count=0
        initial_line_number=NR
        initial_line=$0
        identify_call_or_function_declaration(2)
        while(getline) { 
            identify_call_or_function_declaration(1)
        }  
    } 
    ' employee.txt
