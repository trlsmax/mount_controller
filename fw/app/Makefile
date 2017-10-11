#PROJ_NAME=eqmax

################################################################################
#                   SETUP TOOLS                                                #
################################################################################
BUILD_DIR = .build

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
GDB     = arm-none-eabi-gdb
AS      = arm-none-eabi-as

# directories to be searched for header files
INCLUDE = $(addprefix -I,$(INC_DIRS))

# #defines needed when working with the STM peripherals library
#DEFS    = -DUSE_STDPERIPH_DRIVER
#DEFS   += -DSTM32F10X_MD
#DEFS   += -DDEBUG
#DEFS   += -DMOUNT_ENABLE
#DEFS   += -DFOCUSER_ENABLE

##### Assembler options

AFLAGS  = -mcpu=cortex-m3 
AFLAGS += -mthumb
AFLAGS += -mthumb-interwork
AFLAGS += -mlittle-endian
#AFLAGS += -mfloat-abi=hard
#AFLAGS += -mfpu=fpv4-sp-d16
#AFLAGS += -fsingle-precision-constant

##### Compiler options

CFLAGS  = #-ggdb
CFLAGS += -Os
CFLAGS += -Wall -Wextra -Warray-bounds
CFLAGS += -Wl,-Map,$(BUILD_DIR)/$(PROJ_NAME).map
CFLAGS += $(AFLAGS)
CFLAGS += -ffunction-sections -fno-exceptions -fdata-sections

##### Linker options

# tell ld which linker file to use
# (this file is in the current directory)
LFLAGS  = -Tstm32_flash.ld
LFLAGS += -Wl,-Map,$(BUILD_DIR)/$(PROJ_NAME).map
LFLAGS += -Wl,--gc-sections
#LFLAGS += -Os
LFLAGS += --specs=nano.specs


################################################################################
#                   SOURCE FILES DIRECTORIES                                   #
################################################################################

STM_ROOT         = ../Libraries

STM_SRC_DIR      = $(STM_ROOT)/STM32F10x_StdPeriph_Driver/src
STM_SRC_DIR     += $(STM_ROOT)/STM32_USB-FS-Device_Driver/src
STM_SRC_DIR 	+= $(STM_ROOT)/CMSIS/CM3/CoreSupport/
STM_SRC_DIR 	+= $(STM_ROOT)/CMSIS/CM3/DeviceSupport/ST/STM32F10x/
STM_STARTUP_DIR += $(STM_ROOT)/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/TrueSTUDIO

vpath %.c $(STM_SRC_DIR)
vpath %.s $(STM_STARTUP_DIR)


################################################################################
#                   HEADER FILES DIRECTORIES                                   #
################################################################################

# The header files we use are located here
INC_DIRS  = $(STM_ROOT)/CMSIS/CM3/DeviceSupport/ST/STM32F10x/
INC_DIRS += $(STM_ROOT)/CMSIS/CM3/CoreSupport/
INC_DIRS += $(STM_ROOT)/STM32F10x_StdPeriph_Driver/inc
INC_DIRS += $(STM_ROOT)/STM32_USB-FS-Device_Driver/inc
INC_DIRS += .


################################################################################
#                   SOURCE FILES TO COMPILE                                    #
################################################################################


SRCS  += hw_config.c
SRCS  += stm32_it.c
SRCS  += system_stm32f10x.c
SRCS  += main.c
SRCS  += synta.c
SRCS  += syscall.c

SRCS  += usb_desc.c
SRCS  += usb_endp.c
SRCS  += usb_istr.c
SRCS  += usb_prop.c
SRCS  += usb_pwr.c

SRCS  += stm32f10x_exti.c
SRCS  += stm32f10x_gpio.c
SRCS  += stm32f10x_usart.c
SRCS  += stm32f10x_rcc.c
SRCS  += stm32f10x_tim.c
SRCS  += stm32f10x_flash.c
SRCS  += stm32f10x_spi.c
SRCS  += misc.c

SRCS  += usb_core.c
SRCS  += usb_init.c
SRCS  += usb_int.c
SRCS  += usb_mem.c
SRCS  += usb_regs.c
SRCS  += usb_sil.c

# startup file, calls main
ASRC  = startup_stm32f10x_md.s

_OBJS  = $(SRCS:.c=.o)
_OBJS += $(ASRC:.s=.o)
OBJS = $(patsubst %,$(BUILD_DIR)/%,$(_OBJS))


######################################################################
#                         SETUP TARGETS                              #
######################################################################

.PHONY: all eqmax mount focuser

#all: build_dir $(PROJ_NAME).elf
all: eqmax mount focuser

eqmax: PROJ_NAME = eqmax
eqmax: DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -DMOUNT_ENABLE -DFOCUSER_ENABLE -DFW_TYPE=0x0003
eqmax: build_dir $(PROJ_NAME).elf

mount: PROJ_NAME := mount
mount: DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -DMOUNT_ENABLE -DFW_TYPE=0x0001
mount: build_dir $(PROJ_NAME).elf

focuser: PROJ_NAME = focuser
focuser: DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -DFOCUSER_ENABLE -DUSE_IO -DFW_TYPE=0x0002
focuser: build_dir $(PROJ_NAME).elf

build_dir: $(BUILD_DIR)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o : %.c
	@echo "[Compiling ]  $^"
	@$(CC) -c -o $@ $(INCLUDE) $(DEFS) $(CFLAGS) $^

$(BUILD_DIR)/%.o : %.s
	@echo "[Assembling]  $^"
	@$(AS) $(AFLAGS) $< -o $@


$(PROJ_NAME).elf: $(OBJS)
	@echo "[Linking   ]  $(PROJ_NAME).elf"
	@$(CC) $(CFLAGS) $(LFLAGS) $^ -o $(PROJ_NAME).elf 
	@$(OBJCOPY) -O ihex $(PROJ_NAME).elf   $(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f $(BUILD_DIR)/*.o $(BUILD_DIR)/*.map *.elf *.hex *.bin

flash: all
	st-flash --reset write $(PROJ_NAME).bin 0x8002000

debug:
# before you start gdb, you must start st-util
	$(GDB) -tui $(PROJ_NAME).elf


