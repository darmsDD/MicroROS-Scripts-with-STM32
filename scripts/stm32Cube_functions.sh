#! /bin/bash

SetupStm32CubeIde(){
    
    BASE_DIR=/
    purple_word "Searching for the STM32CubeIde executable"
    stm32cubeideExec=$(find $BASE_DIR -ipath *st/stm32cubeide*/stm32cubeide 2>/dev/null)
    if [[ -n "$stm32cubeideExec" ]]; then
        return 0
    else
        return 1
    fi
}

StartStm32CubeIde(){
    $stm32cubeideExec &>/dev/null &
}

PreBuild(){
    set -x
    project_full_path=$(realpath $micro_utils_folder_path)
    # The full path is needed because if your project has characters, such as _, it returns the error:
    # "your_project_path/project_name" includes invalid characters for a local volume name, only "[a-zA-Z0-9][a-zA-Z0-9_.-]" are allowed. If you intended to pass a host directory, use absolute path.
    docker pull microros/micro_ros_static_library_builder:${ROS_DISTRO} && docker run --rm -v "$project_full_path":/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:${ROS_DISTRO}
    set +x
}


BuildStm32CubeProject(){
    #cd "$micro_utils_folder_path/Debug"
    PreBuild
    pwd
    ls
    build_stm="$stm32cubeideExec --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data "$stm32_workspace_name" -build $stm32_project_name/Debug"
    #clean_build="make"
    #build_stm="make -j8 all"
    #$clean_build
    $build_stm
    cd - &>/dev/null
}


FlashCodeToBoard(){
    #sudo apt install stlink-tools
    #sudo apt install gcc-arm-none-eabi
    arm-none-eabi-objcopy -O binary $stm32_project_name.elf $stm32_project_name.bin
    sudo st-flash write $stm32_project_name.bin 0x08000000
}