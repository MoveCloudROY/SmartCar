################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CODE/shell/src/ansi.c \
../CODE/shell/src/ansi_port.c \
../CODE/shell/src/nr_micro_shell.c 

OBJS += \
./CODE/shell/src/ansi.o \
./CODE/shell/src/ansi_port.o \
./CODE/shell/src/nr_micro_shell.o 

COMPILED_SRCS += \
./CODE/shell/src/ansi.src \
./CODE/shell/src/ansi_port.src \
./CODE/shell/src/nr_micro_shell.src 

C_DEPS += \
./CODE/shell/src/ansi.d \
./CODE/shell/src/ansi_port.d \
./CODE/shell/src/nr_micro_shell.d 


# Each subdirectory must supply rules for building sources it contributes
CODE/shell/src/%.src: ../CODE/shell/src/%.c CODE/shell/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fD:/Code/Embedded/SmartCar/GoAhead2022/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

CODE/shell/src/%.o: ./CODE/shell/src/%.src CODE/shell/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


