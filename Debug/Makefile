################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../main.c \
../motion_control.c \
../gcode.c \
../spindle_control.c \
../coolant_control.c \
../serial.c \
../protocol.c \
../stepper.c \
../eeprom.c \
../settings.c \
../planner.c \
../nuts_bolts.c \
../limits.c \
../print.c \
../probe.c \
../report.c \
../system.c \
../status.c \
../i2c_master.c \
../tool_changer.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
main.o \
motion_control.o \
gcode.o \
spindle_control.o \
coolant_control.o \
serial.o \
protocol.o \
stepper.o \
eeprom.o \
settings.o \
planner.o \
nuts_bolts.o \
limits.o \
print.o \
probe.o \
report.o \
system.o \
status.o \
i2c_master.o \
tool_changer.o

OBJS_AS_ARGS +=  \
main.o \
motion_control.o \
gcode.o \
spindle_control.o \
coolant_control.o \
serial.o \
protocol.o \
stepper.o \
eeprom.o \
settings.o \
planner.o \
nuts_bolts.o \
limits.o \
print.o \
probe.o \
report.o \
system.o \
status.o \
i2c_master.o \
tool_changer.o

C_DEPS +=  \
main.d \
motion_control.d \
gcode.d \
spindle_control.d \
coolant_control.d \
serial.d \
protocol.d \
stepper.d \
eeprom.d \
settings.d \
planner.d \
nuts_bolts.d \
limits.d \
print.d \
probe.d \
report.d \
system.d \
status.d \
i2c_master.d \
tool_changer.d

C_DEPS_AS_ARGS +=  \
main.d \
motion_control.d \
gcode.d \
spindle_control.d \
coolant_control.d \
serial.d \
protocol.d \
stepper.d \
eeprom.d \
settings.d \
planner.d \
nuts_bolts.d \
limits.d \
print.d \
probe.d \
report.d \
system.d \
status.d \
i2c_master.d \
tool_changer.d

OUTPUT_FILE_PATH +=mschorer-grbl.elf

OUTPUT_FILE_PATH_AS_ARGS +=mschorer-grbl.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

# AVR32/GNU C Compiler









































./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega328p -c -std=gnu99 -DF_CPU=16000000 -mmcu=atmega328p -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="mschorer-grbl.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=atmega328p  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "mschorer-grbl.elf" "mschorer-grbl.hex"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "mschorer-grbl.elf" "mschorer-grbl.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "mschorer-grbl.elf" > "mschorer-grbl.lss"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "mschorer-grbl.elf" "mschorer-grbl.srec"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-size.exe" "mschorer-grbl.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "mschorer-grbl.elf" "mschorer-grbl.a" "mschorer-grbl.hex" "mschorer-grbl.lss" "mschorer-grbl.eep" "mschorer-grbl.map" "mschorer-grbl.srec" "mschorer-grbl.usersignatures"
	