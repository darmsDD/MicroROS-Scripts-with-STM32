#! /bin/bash

#=================================================================  Multiline Variables========================================================================================================================================
#==============================================================================================================================================================================================================================

# All multiline variables will be appended to the ioc file in their correct positions.


dma_var="/CAD.provider=/a\Dma.Request0=USART3_RX
Dma.Request1=USART3_TX
Dma.RequestsNb=2
Dma.USART3_RX.0.Direction=DMA_PERIPH_TO_MEMORY
Dma.USART3_RX.0.EventEnable=DISABLE
Dma.USART3_RX.0.Instance=DMA1_Channel1
Dma.USART3_RX.0.MemDataAlignment=DMA_MDATAALIGN_BYTE
Dma.USART3_RX.0.MemInc=DMA_MINC_ENABLE
Dma.USART3_RX.0.Mode=DMA_CIRCULAR
Dma.USART3_RX.0.PeriphDataAlignment=DMA_PDATAALIGN_BYTE
Dma.USART3_RX.0.PeriphInc=DMA_PINC_DISABLE
Dma.USART3_RX.0.Polarity=HAL_DMAMUX_REQ_GEN_RISING
Dma.USART3_RX.0.Priority=DMA_PRIORITY_VERY_HIGH
Dma.USART3_RX.0.RequestNumber=1
Dma.USART3_RX.0.RequestParameters=Instance,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,SignalID,Polarity,RequestNumber,SyncSignalID,SyncPolarity,SyncEnable,EventEnable,SyncRequestNumber
Dma.USART3_RX.0.SignalID=NONE
Dma.USART3_RX.0.SyncEnable=DISABLE
Dma.USART3_RX.0.SyncPolarity=HAL_DMAMUX_SYNC_NO_EVENT
Dma.USART3_RX.0.SyncRequestNumber=1
Dma.USART3_RX.0.SyncSignalID=NONE
Dma.USART3_TX.1.Direction=DMA_MEMORY_TO_PERIPH
Dma.USART3_TX.1.EventEnable=DISABLE
Dma.USART3_TX.1.Instance=DMA1_Channel2
Dma.USART3_TX.1.MemDataAlignment=DMA_MDATAALIGN_BYTE
Dma.USART3_TX.1.MemInc=DMA_MINC_ENABLE
Dma.USART3_TX.1.Mode=DMA_NORMAL
Dma.USART3_TX.1.PeriphDataAlignment=DMA_PDATAALIGN_BYTE
Dma.USART3_TX.1.PeriphInc=DMA_PINC_DISABLE
Dma.USART3_TX.1.Polarity=HAL_DMAMUX_REQ_GEN_RISING
Dma.USART3_TX.1.Priority=DMA_PRIORITY_VERY_HIGH
Dma.USART3_TX.1.RequestNumber=1
Dma.USART3_TX.1.RequestParameters=Instance,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,SignalID,Polarity,RequestNumber,SyncSignalID,SyncPolarity,SyncEnable,EventEnable,SyncRequestNumber
Dma.USART3_TX.1.SignalID=NONE
Dma.USART3_TX.1.SyncEnable=DISABLE
Dma.USART3_TX.1.SyncPolarity=HAL_DMAMUX_SYNC_NO_EVENT
Dma.USART3_TX.1.SyncRequestNumber=1
Dma.USART3_TX.1.SyncSignalID=NONE"


freertos_var="/Dma.USART3_TX.1.SyncSignalID=/a\FREERTOS.FootprintOK=true
FREERTOS.IPParameters=Tasks01,configUSE_NEWLIB_REENTRANT,configTOTAL_HEAP_SIZE,FootprintOK
FREERTOS.Tasks01=defaultTask,24,3000,StartDefaultTask,Default,NULL,Dynamic,NULL,NULL
FREERTOS.configTOTAL_HEAP_SIZE=15000
FREERTOS.configUSE_NEWLIB_REENTRANT=1"

nvic_var1="/NVIC.BusFault_IRQn=/a\NVIC.DMA1_Channel1_IRQn=true\\\:5\\\:0\\\:false\\\:false\\\:true\\\:true\\\:false\\\:true\\\:true
NVIC.DMA1_Channel2_IRQn=true\\\:5\\\:0\\\:false\\\:false\\\:true\\\:true\\\:false\\\:true\\\:true"


nvic_var2="/NVIC.SVCall_IRQn=/a\NVIC.SavedPendsvIrqHandlerGenerated=true
NVIC.SavedSvcallIrqHandlerGenerated=true
NVIC.SavedSystickIrqHandlerGenerated=true"

nvic_var3="/NVIC.SysTick_IRQn=/a\NVIC.TIM6_DAC_IRQn=true\\\:15\\\:0\\\:false\\\:false\\\:true\\\:false\\\:false\\\:true\\\:true
NVIC.TimeBase=TIM6_DAC_IRQn
NVIC.TimeBaseIP=TIM6
NVIC.UART4_IRQn=true\\\:5\\\:0\\\:false\\\:false\\\:true\\\:true\\\:true\\\:true\\\:true
NVIC.USART3_IRQn=true\\\:5\\\:0\\\:false\\\:false\\\:true\\\:true\\\:true\\\:true\\\:true"

pb_var="/PA5.Signal=/a\PB10.Mode=Asynchronous
PB10.Signal=USART3_TX
PB11.Mode=Asynchronous
PB11.Signal=USART3_RX"

pc_var="/PB3.Signal=/a\PC10.Mode=Asynchronous
PC10.Signal=UART4_TX
PC11.Mode=Asynchronous
PC11.Signal=UART4_RX"


usart3_var="/SH.GPXTI13.ConfNb=/a\USART3.IPParameters=VirtualMode-Asynchronous
USART3.VirtualMode-Asynchronous=VM_ASYNC"


freertos_cmsis_var="/USART3.VirtualMode-Asynchronous=/a\VP_FREERTOS_VS_CMSIS_V2.Mode=CMSIS_V2
VP_FREERTOS_VS_CMSIS_V2.Signal=FREERTOS_VS_CMSIS_V2"


#"$nvic_var3" 
# Store variables in an array
multiple_lines_append=("$dma_var" "$freertos_var" "$nvic_var1" "$nvic_var2" "$nvic_var3" "$pb_var" "$pc_var" "$usart3_var" "$freertos_cmsis_var")

#================================================================= Single Line Variables ========================================================================================================================================
#==============================================================================================================================================================================================================================

#=================== Substitute ===========================

# Array for substitute commands
substitute_commands=(
    '/Mcu.IP0=/s/=.*$/=DMA/'
    '/Mcu.IP1=/s/=.*$/=FREERTOS/'
    '/Mcu.IP2=/s/=.*$/=NVIC/'
    '/Mcu.IP3=/s/=.*$/=RCC/'
    '/Mcu.IPNb=/s/=.*$/=8/'
    '/Mcu.Pin10=/s/=.*$/=PA13/'
    '/Mcu.Pin11=/s/=.*$/=PA14/'
    '/Mcu.Pin12=/s/=.*$/=PC10/'
    '/Mcu.Pin8=/s/=.*$/=PB10/'
    '/Mcu.Pin9=/s/=.*$/=PB11/'
    '/Mcu.PinsNb=/s/=.*$/=18/'
    '/NVIC.BusFault_IRQn=/s/=.*$/=true\\\:0\\\:0\\\:false\\\:false\\\:true\\\:false\\\:false\\\:false\\\:false/'
    '/NVIC.DebugMonitor_IRQn=/s/=.*$/=true\\\:0\\\:0\\\:false\\\:false\\\:true\\\:false\\\:false\\\:false\\\:false/'
    '/NVIC.HardFault_IRQn=/s/=.*$/=true\\\:0\\\:0\\\:false\\\:false\\\:true\\\:false\\\:false\\\:false\\\:false/'
    '/NVIC.MemoryManagement_IRQn=/s/=.*$/=true\\\:0\\\:0\\\:false\\\:false\\\:true\\\:false\\\:false\\\:false\\\:false/'
    '/NVIC.NonMaskableInt_IRQn=/s/=.*$/=true\\\:0\\\:0\\\:false\\\:false\\\:true\\\:false\\\:false\\\:false\\\:false/'
    '/NVIC.PendSV_IRQn=/s/true\\\:0\\\:0/true\\\:15\\\:0\\\:false/'
    '/NVIC.SVCall_IRQn=/s/=.*$/=true\\\:0\\\:0\\\:false\\\:false\\\:false\\\:false\\\:false\\\:false\\\:false/'
    '/NVIC.EXTI15_10_IRQn=/s/=.*$/=true\\\:5\\\:0\\\:false\\\:false\\\:true\\\:true\\\:true\\\:true\\\:true/' 
    '/NVIC.UsageFault_IRQn=/s/=.*$/=true\\\:0\\\:0\\\:false\\\:false\\\:true\\\:false\\\:false\\\:false\\\:false/'
    '/ProjectManager.functionlistsort=/s/=.*$/=1-SystemClock_Config-RCC-false-HAL-false,2-MX_GPIO_Init-GPIO-false-HAL-true,3-MX_DMA_Init-DMA-false-HAL-true,4-MX_USART3_UART_Init-USART3-false-HAL-true,5-MX_UART4_Init-UART4-false-HAL-true/'
    '/VP_SYS_VS_Systick.Mode=/s/Systick.Mode=SysTick/tim6.Mode=TIM6/'
    '/VP_SYS_VS_Systick.Signal=/s/Systick.Signal=SYS_VS_Systick/tim6.Signal=SYS_VS_tim6/'
    '/NVIC.SysTick_IRQn=/s/true\\\:0\\\:0/true\\\:15\\\:0\\\:false/'
    '/ProjectManager.CoupleFile=/s/false/true/'
)


#==================== Append ==============================

# Array for append commands
append_commands=(
    '/Mcu.IP3=/a\Mcu.IP4=SYS'
    '/Mcu.IP4=/a\Mcu.IP5=UART4'
    '/Mcu.IP5=/a\Mcu.IP6=USART3'
    '/Mcu.IP6=/a\Mcu.IP7=NUCLEO-G474RE'
    '/Mcu.Pin12=/a\Mcu.Pin13=PC11'
    '/Mcu.Pin13=/a\Mcu.Pin14=PB3'
    '/Mcu.Pin14=/a\Mcu.Pin15=VP_FREERTOS_VS_CMSIS_V2'
    '/Mcu.Pin15=/a\Mcu.Pin16=VP_SYS_VS_tim6'
    '/Mcu.Pin16=/a\Mcu.Pin17=VP_SYS_VS_DBSignals'
    '/boardIOC=/a\rtos.0.ip=FREERTOS'
)

#====================================== Others =========================================================


# Array for other commands
other_commands=(
    '/NVIC.TIM6_DAC_IRQn=/c\NVIC.TIM6_DAC_IRQn=true\\\:15\\\:0\\\:false\\\:false\\\:true\\\:false\\\:false\\\:true\\\:true'
)