PROJECT = seismo
TARGET = $(PROJECT).bin
EXECUTABLE = $(PROJECT).elf

CC=arm-none-eabi-gcc
LD=arm-none-eabi-ld 
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump

#DEFS = -DSTM32F40_41xxx -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000UL

MCU = cortex-m4
MCFLAGS = 	-ggdb  \
			-mcpu=$(MCU)  \
			-mthumb  \
			-mlittle-endian  \
			-mfpu=fpv4-sp-d16  \
			-mfloat-abi=hard  \
			-mthumb-interwork  \
			-nostartfiles \
			-Xlinker -Map=$(PROJECT).map

HAL_DIR = $(HOME)/STM32Cube/Repository/STM32Cube_FW_F4_V1.5.0
CMSIS_DIR = $(HAL_DIR)/Drivers/CMSIS
CMSIS_CORE_INC = $(CMSIS_DIR)/Include
CMSIS_CORE_INC_LEGACY = $(CMSIS_CORE_INC)/Legacy

CMSIS_DEVICE_INC = $(CMSIS_DIR)/Device/ST/STM32F4xx/Include
PERIPHERAL_INC = $(HAL_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc
PERIPHERAL_SRC = $(HAL_DIR)/Drivers/STM32F4xx_HAL_Driver/Src
FREERTOS = $(HAL_DIR)/Middlewares/Third_Party/FreeRTOS
FREERTOS_INC = $(FREERTOS)/Source/include
FREERTOS_CMSIS = $(FREERTOS)/Source/CMSIS_RTOS
FREERTOS_PORT = $(FREERTOS)/Source/portable/GCC/ARM_CM4F

PROJECT_DIR = .
PROJECT_INC = $(PROJECT_DIR)/Inc
PROJECT_SRC = $(PROJECT_DIR)/Src

INCLUDES = 	-I$(CMSIS_CORE_INC) \
 			-I$(CMSIS_DEVICE_INC)  \
 			-I$(CMSIS_DEVICE_INC)  \
 			-I$(PERIPHERAL_INC)  \
 			-I$(PROJECT_INC) \
 			-I$(FREERTOS_INC) \
 			-I$(FREERTOS_CMSIS) \
 			-I$(FREERTOS_PORT)

OPTIMIZE = -O0

LDSCRIPT = $(PROJECT_DIR)/stm32f4xx_flash.ld

CFLAGS	= $(MCFLAGS)  $(OPTIMIZE)  $(INCLUDES) -Wall -T $(LDSCRIPT) -DSTM32F407xx -DUSE_HAL_DRIVER
AFLAGS	= $(MCFLAGS) 

MIDDLEWARES_SRC = 		$(FREERTOS)/Source/portable/MemMang/heap_4.c \
						$(FREERTOS)/Source/list.c \
						$(FREERTOS)/Source/tasks.c \
						$(FREERTOS)/Source/event_groups.c \
						$(FREERTOS)/Source/queue.c \
						$(FREERTOS)/Source/croutine.c \
						$(FREERTOS)/Source/timers.c \
				 	    $(FREERTOS)/Source/CMSIS_RTOS/cmsis_os.c \
						$(FREERTOS)/Source/portable/GCC/ARM_CM4F/port.c 
						# $(FREERTOS)/Source/portable/GCC/ARM_CM4F/portasm.s 

DRIVERS_SRC =			$(PERIPHERAL_SRC)/stm32f4xx_hal_flash.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_flash_ex.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_pwr.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_pwr_ex.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_gpio.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_dma.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_adc.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_adc_ex.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_dma_ex.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_spi.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_tim.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_tim_ex.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_uart.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_rcc.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_rcc_ex.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_cortex.c \
						$(PERIPHERAL_SRC)/stm32f4xx_hal_flash_ramfunc.c 

APPLICATION_SRC =       $(PROJECT_SRC)/main.c \
				        $(PROJECT_SRC)/freertos.c \
				        $(PROJECT_SRC)/stm32f4xx_it.c \
				        $(PROJECT_SRC)/stm32f4xx_hal_msp.c  \
				        $(PROJECT_SRC)/system_stm32f4xx.c 

STARTUP = $(PROJECT_SRC)/startup_stm32f4xx.s

SRC = $(MIDDLEWARES_SRC) $(DRIVERS_SRC) $(APPLICATION_SRC) $(STARTUP)

all: $(TARGET)

$(TARGET): $(EXECUTABLE)
	$(CP) -O binary $^ $@

$(EXECUTABLE): $(SRC)
	$(CC) $(CFLAGS) $^ -o $@

.PHONY: clean burn debug

burn:
	@./st-flash write $(TARGET) 0x8000000

debug:
	@./st-util>/dev/null 2>&1 &
	@arm-none-eabi-gdb --command=gdb-init

clean:
	@rm -f $(TARGET) $(EXECUTABLE)