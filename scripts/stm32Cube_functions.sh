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
    local if_no_lib_string=$5
    #set -x
    if  ! grep -q "$line_to_append" "$XML_FILE"; then
        if ! grep -q "$search_for" "$XML_FILE"; then
            sed -i "/Linker Script (-T)/ a\\$if_no_lib_string" $XML_FILE
        else  
            sed -i "/$search_for/ {
                    s/\/>/>/
                    a \ $tab_n_times$line_to_append
                    /IS_VALUE_EMPTY=\"true\"/ a\ $second_line_to_append
                    /IS_VALUE_EMPTY=\"false\"/! s/IS_VALUE_EMPTY=\"true\"/IS_VALUE_EMPTY=\"false\"/ 
            }" $XML_FILE
        fi
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
    
    # ========================= Project properties ->C/C++ Build -> Settings ->  MCU/MPU GCC COMPILER -> Include Paths -> Include path(-I) ==============
     # ==================================================================================================================================================

    line_to_append="<listOptionValue builtIn=\"false\" value=\"&quot;\${PWD}/../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include&quot;\"/>"
    interval_open="<folderInfo id=\"com.st.stm32cube.ide.mcu.gnu.managedbuild.config.exe.debug."
    interval_close="<\/folderInfo>"
    search_for="Include paths (-I)"
    if ! grep -q "$line_to_append" "$XML_FILE"; then
        sed -i "/$interval_open[^>]*>/,/$interval_open_close/ {
        /$search_for/ a\ $tab_9_times$line_to_append
        }" $XML_FILE
    fi

    # ========================= Project properties ->C/C++ Build -> Settings ->  MCU/MPU GCC LINKER -> Libraries -> Library search path(-L) ============
    # ==================================================================================================================================================
    second_line_to_append="$tab_8_times</option>"
    library_path_random_number=$(BaseFunctions_GenerateRandomNumber)
    library_search_path="$tab_8_times<option IS_BUILTIN_EMPTY=\"false\" IS_VALUE_EMPTY=\"false\" id=\"com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.linker.option.directories.$library_path_random_number\" name=\"Library search path (-L)\" superClass=\"com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.linker.option.directories\" valueType=\"libPaths\">
$tab_9_times<listOptionValue builtIn=\"false\" value=\"&quot;\${PWD}/../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros&quot;\"/>
$tab_8_times</option>"

    escaped_string=$(echo "$library_search_path" | sed 's/$/\\/' | sed '$ s/\\$//')  # add \ to end of line in every lane. Then removes the \ in the last line. This is needed to work with sed and strings with multiple lines.
    line_to_append="<listOptionValue builtIn=\"false\" value=\"&quot;\${PWD}/../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros&quot;\"/>"
    search_for="Library search path (-L)"
    STM32CUBE_AddMicrorosLibToCProject "$line_to_append" "$search_for" "$second_line_to_append" 9 "$escaped_string"
    

    # ========================= Project properties ->C/C++ Build -> Settings ->  MCU/MPU GCC LINKER -> Libraries -> Libraries(-l) ============
    # ==================================================================================================================================================
    library_random_number=$(BaseFunctions_GenerateRandomNumber)
    library_microros="$tab_8_times<option IS_BUILTIN_EMPTY=\"false\" IS_VALUE_EMPTY=\"false\" id=\"com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.linker.option.libraries.$library_random_number\" name=\"Libraries (-l)\" superClass=\"com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.linker.option.libraries\" valueType=\"libs\">
$tab_9_times<listOptionValue builtIn=\"false\" value=\"microros\"/>
$tab_8_times</option>"

    escaped_string=$(echo "$library_microros" | sed 's/$/\\/' | sed '$ s/\\$//') # add \ to end of line in every lane. Then removes the \ in the last line. This is needed to work with sed and strings with multiple lines.
    line_to_append="<listOptionValue builtIn=\"false\" value=\"microros\"/>"
    search_for="Libraries (-l)"
    STM32CUBE_AddMicrorosLibToCProject "$line_to_append" "$search_for" "$second_line_to_append" 9 "$escaped_string"
}


STM32Cube_AlterIOCProperties(){

    file=$micro_utils_folder_path/teste.ioc
    var="FREERTOS.IPParameters=Tasks01,configUSE_NEWLIB_REENTRANT
FREERTOS.Tasks01=defaultTask,24,128,StartDefaultTask,Default,NULL,Dynamic,NULL,NULL
FREERTOS.configUSE_NEWLIB_REENTRANT=1"
    escaped_var=$(echo "$var" | sed 's/$/\\/' | sed '$ s/\\$//')
    sed -i "/CAD.provider=/ a\\$escaped_var" $file


    sed -i "/Mcu.IP0=/s/NVIC/FREERTOS/" $file
    sed -i "/Mcu.IP1=/s/RCC/NVIC/" $file
    sed -i "/Mcu.IP2=/s/SYS/RCC/" $file
    sed -i "/Mcu.IP3=/s/NUCLEO-G474RE/SYS/" $file
    sed -i "/Mcu.IP3=/a\Mcu.IP4=NUCLEO-G474RE" $file
    sed -i "/Mcu.IPNb=/s/4/5/" $file

    sed -i "/Mcu.Pin11=/s/VP_SYS_VS_Systick/VP_FREERTOS_VS_CMSIS_V2/" $file
    sed -i "/Mcu.Pin12=/s/VP_SYS_VS_DBSignals/VP_SYS_VS_tim6/" $file
    sed -i "/Mcu.Pin12=/a\Mcu.Pin13=VP_SYS_VS_DBSignals" $file
    sed -i "/Mcu.PinsNb=/s/13/14/" $file
    sed -i "/NVIC.BusFault_IRQn=/s|$|\\\:false|" $file
    sed -i "/NVIC.DebugMonitor_IRQn=/s|$|\\\:false|" $file
   
    sed -i "/NVIC.EXTI15_10_IRQn=/ {
                s/true\\\:0/true\\\:5/
                s|$|\\\:true|
            }" $file
    

    sed -i "/NVIC.HardFault_IRQn=/s|$|\\\:false|" $file
    sed -i "/NVIC.MemoryManagement_IRQn=/s|$|\\\:false|" $file
    sed -i "/NVIC.NonMaskableInt_IRQn=/s|$|\\\:false|" $file
    sed -i "/NVIC.PendSV_IRQn=/s|true\\\:0\\\:0|true\\\:15\\\:0\\\:false|" $file


    sed -i "/NVIC.SVCall_IRQn=/ {
                s/:true/:false/
                s|$|\\\:false|
            }" $file
    
    var2="NVIC.SavedPendsvIrqHandlerGenerated=true
NVIC.SavedSvcallIrqHandlerGenerated=true
NVIC.SavedSystickIrqHandlerGenerated=true"
    escaped_var=$(echo "$var2" | sed 's/$/\\/' | sed '$ s/\\$//')
    sed -i "/NVIC.SVCall_IRQn=/ a\\$escaped_var" $file



    var3="NVIC.TIM6_DAC_IRQn=true\:15\:0\:false\:false\:true\:false\:false\:true\:true
NVIC.TimeBase=TIM6_DAC_IRQn
NVIC.TimeBaseIP=TIM6"
    escaped_var=$(echo "$var3" | sed 's/$/\\/' | sed '$ s/\\$//')
    sed -i "/NVIC.SysTick_IRQn=/{
                s|true\\\:0\\\:0|true\\\:15\\\:0\\\:false|
                a\\$escaped_var
            }" $file
    
    sed -i "/NVIC.TIM6_DAC_IRQn=/c\NVIC.TIM6_DAC_IRQn=true\\\:15\\\:0\\\:false\\\:false\\\:true\\\:false\\\:false\\\:true\\\:true" $file
    sed -i "/NVIC.UsageFault_IRQn=/s|$|\\\:false|" $file


    var4="VP_FREERTOS_VS_CMSIS_V2.Mode=CMSIS_V2
VP_FREERTOS_VS_CMSIS_V2.Signal=FREERTOS_VS_CMSIS_V2"
    escaped_var=$(echo "$var4" | sed 's/$/\\/' | sed '$ s/\\$//')
    sed -i "/SH.GPXTI13.ConfNb=/a\\$escaped_var" $file
    
    sed -i "/VP_SYS_VS_Systick.Mode=/s/Systick.Mode=SysTick/tim6.Mode=TIM6/" $file
    sed -i "/VP_SYS_VS_Systick.Signal=/s/Systick.Signal=SYS_VS_Systick/tim6.Signal=SYS_VS_tim6/" $file
    sed -i "/isbadioc=false/d" $file
     



}