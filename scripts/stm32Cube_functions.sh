#! /bin/bash

STM32Cube_Setup(){
    
    BASE_DIR=/
    Style_PurpleWord "Searching for the STM32CubeIde executable"
    stm32cubeideExec=$(find $BASE_DIR -ipath *st/stm32cubeide*/stm32cubeide 2>/dev/null)
    if [[ -n "$stm32cubeideExec" ]]; then
        return 0
    else
        return 1
    fi
}

STM32Cube_StartIde(){
    $stm32cubeideExec &>/dev/null &
}

STM32Cube_PrebuildDocker(){
    project_full_path=$(realpath $micro_utils_folder_path)
    # The full path is needed because if your project has characters, such as _, it returns the error:
    # "your_project_path/project_name" includes invalid characters for a local volume name, only "[a-zA-Z0-9][a-zA-Z0-9_.-]" are allowed. If you intended to pass a host directory, use absolute path.
    echo "docker pull microros/micro_ros_static_library_builder:${ROS_DISTRO} && docker run --rm -v "$project_full_path":/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:${ROS_DISTRO}"
    # docker pull microros/micro_ros_static_library_builder:${ROS_DISTRO} && docker run --rm -v "$project_full_path":/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:${ROS_DISTRO}
}


STM32Cube_BuildProject(){
    #cd "$micro_utils_folder_path/Debug"
    build_stm="$stm32cubeideExec --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data "$stm32_workspace_name" -build $stm32_project_name/Debug"
    #clean_build="make"
    #build_stm="make -j8 all"
    #$clean_build
    $build_stm
    # cd - &>/dev/null
    
}


STM32Cube_FlashCodeToBoard(){
    #sudo apt install stlink-tools
    #sudo apt install gcc-arm-none-eabi
    arm-none-eabi-objcopy -O binary $stm32_project_name.elf $stm32_project_name.bin
    sudo st-flash write $stm32_project_name.bin 0x08000000
}


STM32CUBE_AddMicrorosLibToCProject(){
    local line_to_append=$1
    local search_for=$2
    local second_line_to_append=$3
    local tab_n_times=$(Style_RepeatChar "\t" $4)
    if ! grep -q "$line_to_append" "$XML_FILE"; then
    sed -i "/$search_for/ {
            s/\/>/>/
            a \ $tab_n_times$line_to_append
            /IS_VALUE_EMPTY=\"true\"/ a\ $second_line_to_append
            /IS_VALUE_EMPTY=\"false\"/! s/IS_VALUE_EMPTY=\"true\"/IS_VALUE_EMPTY=\"false\"/ 
        }" $XML_FILE
    fi
    set +x
}


STM32Cube_AlterProjectProperties(){
    # Step 5 of Bacurau's guide.
    : '
        5. Adicione o caminho do micro-ROS nos “includes”. Vá em Project -> Settings -> C/C++ Build -> Settings -> Tool Settings Tab -> MCU GCC Compiler -> Include paths e adicione:
        ${PWD}/../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
    '
    
    file_path="$micro_utils_folder_path/.cproject"
    XML_FILE=$file_path
    tab_9_times=$(Style_RepeatChar "\t" 9)
    tab_8_times=$(Style_RepeatChar "\t" 8)
    
    line_to_append="<listOptionValue builtIn=\"false\" value=\"&quot;\${PWD}/../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include&quot;\"/>"
    interval_open="<folderInfo id=\"com.st.stm32cube.ide.mcu.gnu.managedbuild.config.exe.debug."
    interval_close="<\/folderInfo>"
    search_for="Include paths (-I)"
    if ! grep -q "$line_to_append" "$XML_FILE"; then
        sed -i "/$interval_open[^>]*>/,/$interval_open_close/ {
        /$search_for/ a\ $tab_9_times$line_to_append
        }" $XML_FILE
    fi

    line_to_append="<listOptionValue builtIn=\"false\" value=\"microros\"/>"
    search_for="Libraries (-l)"
    second_line_to_append="$tab_8_times</option>"
    STM32CUBE_AddMicrorosLibToCProject "$line_to_append" "$search_for" "$second_line_to_append" 9

    line_to_append="<listOptionValue builtIn=\"false\" value=\"&quot;\${PWD}/../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros&quot;\"/>"
    search_for="Library search path (-L)"
    STM32CUBE_AddMicrorosLibToCProject "$line_to_append" "$search_for" "$second_line_to_append" 9
    
    
}

#pwd; cd ../../../scripts;./main.sh;

# grep -q '&quot;${PWD}/../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include&quot;' || sed -i '/<option IS_BUILTIN_EMPTY="false" IS_VALUE_EMPTY="false" id="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.option.includepaths.2063006509"/a\
# \t\t\t\t\t\t\t\t\t<listOptionValue builtIn="false" value="&quot;${PWD}/../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include&quot;"/>' $micro_utils_folder_path/.cproject
#