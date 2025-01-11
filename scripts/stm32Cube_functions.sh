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



STM32Cube_AlterProjectProperties(){
    # Step 5 of Bacurau's guide.
    : '
        5. Adicione o caminho do micro-ROS nos “includes”. Vá em Project -> Settings -> C/C++ Build -> Settings -> Tool Settings Tab -> MCU GCC Compiler -> Include paths e adicione:
        ${PWD}/../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
    '
    pwd
    cubemx_utils_line="<listOptionValue builtIn=\"false\" value=\"\&quot;\${PWD}/../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include\&quot;\"/>"
    sed_search_pattern='<option IS_BUILTIN_EMPTY="false" IS_VALUE_EMPTY="false" id="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.option.includepaths.2063006509" name="Include paths (-I)" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.option.includepaths" useByScannerDiscovery="false" valueType="includePath">'
    # If line is not on the file .cproject, insert line.
    # sed -i /pattern/a\subsitution . Search for pattern, appends (a), then replace it with your new sentence. The -i flag is to avoid printing to console.
    grep -q "$cubemx_utils_line" stm32_workspace/stm32_project/.cproject || sed -i /"$sed_search_pattern"/a\\"\t\t\t\t\t\t\t\t\t$cubemx_utils_line" stm32_workspace/stm32_project/.cproject
    Style_PurpleWord "I am here fuckers"

    
    microros_lib_line="<listOptionValue builtIn=\"false\" value=\"microros\"/>"
    line_above_microros_lib_line_v1="<option IS_BUILTIN_EMPTY=\"false\" IS_VALUE_EMPTY=\"true\" id=\"com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.linker.option.libraries.928418063\" name=\"Libraries (-l)\" superClass=\"com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.linker.option.libraries\" valueType=\"libs\""
    line_above_microros_lib_line_v2="<option IS_BUILTIN_EMPTY=\"false\" IS_VALUE_EMPTY=\"false\" id=\"com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.linker.option.libraries.928418063\" name=\"Libraries (-l)\" superClass=\"com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.linker.option.libraries\" valueType=\"libs\""
    line_under_microros_lib_line="</option>" 
    file_path="stm32_workspace/stm32_project/.cproject"

    # Adding tabs for prettier format"
    # For some reason, the first \char prints the char instead of the function.
    # For example: \t prints t instead of tab.
    microros_lib_line_with_tab="\ \t\t\t\t\t\t\t\t\t${microros_lib_line}"
    line_under_microros_lib_line_with_tab="\ \t\t\t\t\t\t\t\t${line_under_microros_lib_line}"


    # Check if microros line exists, if it does not:
    # 1: Check if there are other libraries, if there are execute elif
    #   Just adds the microros lib
    # 2: If there are no other libraries, execute the if.
    #   Replaces the line where the libs are placed.
    #   IS_VALUE_EMPTY="true" => IS_VALUE_EMPTY="false"
    #   And then adds the microros lib 

    if ! grep -q "$microros_lib_line" "$file_path"; then
        if grep -q "$line_above_microros_lib_line_v1" "$file_path"; then
            # Replace the line and insert a new line under it
            sed -i "/$line_above_microros_lib_line_v1/ {
                s|$line_above_microros_lib_line_v1/|$line_above_microros_lib_line_v2|
                a $microros_lib_line_with_tab
                a $line_under_microros_lib_line_with_tab
            }" "$file_path"
            echo "Line replaced and new line inserted."
        elif grep -q "$line_above_microros_lib_line_v2" "$file_path"; then
            sed -i "/$line_above_microros_lib_line_v2/a\\\t\t\t\t\t\t\t\t\t\t$microros_lib_line" "$file_path"
            echo "Other Line replaced and new line inserted."
        else
            echo "Line not found."
        fi
    fi
    
    
    
    
    
    #sed_search_pattern2='<option IS_BUILTIN_EMPTY="false" IS_VALUE_EMPTY="true" id="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.linker.option.libraries.928418063" name="Libraries (-l)" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.linker.option.libraries" valueType="libs"/>'
    #grep -q "$lib_line" stm32_workspace/stm32_project/.cproject || sed -i /"$sed_search_pattern2"/a\\"\t\t\t\t\t\t\t\t\t$lib_line" stm32_workspace/stm32_project/.cproject
    set +x
}

#pwd; cd ../../../scripts;./main.sh;

# grep -q '&quot;${PWD}/../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include&quot;' || sed -i '/<option IS_BUILTIN_EMPTY="false" IS_VALUE_EMPTY="false" id="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.option.includepaths.2063006509"/a\
# \t\t\t\t\t\t\t\t\t<listOptionValue builtIn="false" value="&quot;${PWD}/../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include&quot;"/>' $micro_utils_folder_path/.cproject
#