################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CODE/ESKF.c \
../CODE/ImageDeal.c \
../CODE/ImagePreDeal.c \
../CODE/KF.c \
../CODE/data.c \
../CODE/key.c \
../CODE/motor.c \
../CODE/pid.c \
../CODE/shell.c \
../CODE/steer.c \
../CODE/system.c \
../CODE/ui.c \
../CODE/vofa.c \
../CODE/vt100.c \
../CODE/yawAngle.c 

OBJS += \
./CODE/ESKF.o \
./CODE/ImageDeal.o \
./CODE/ImagePreDeal.o \
./CODE/KF.o \
./CODE/data.o \
./CODE/key.o \
./CODE/motor.o \
./CODE/pid.o \
./CODE/shell.o \
./CODE/steer.o \
./CODE/system.o \
./CODE/ui.o \
./CODE/vofa.o \
./CODE/vt100.o \
./CODE/yawAngle.o 

COMPILED_SRCS += \
./CODE/ESKF.src \
./CODE/ImageDeal.src \
./CODE/ImagePreDeal.src \
./CODE/KF.src \
./CODE/data.src \
./CODE/key.src \
./CODE/motor.src \
./CODE/pid.src \
./CODE/shell.src \
./CODE/steer.src \
./CODE/system.src \
./CODE/ui.src \
./CODE/vofa.src \
./CODE/vt100.src \
./CODE/yawAngle.src 

C_DEPS += \
./CODE/ESKF.d \
./CODE/ImageDeal.d \
./CODE/ImagePreDeal.d \
./CODE/KF.d \
./CODE/data.d \
./CODE/key.d \
./CODE/motor.d \
./CODE/pid.d \
./CODE/shell.d \
./CODE/steer.d \
./CODE/system.d \
./CODE/ui.d \
./CODE/vofa.d \
./CODE/vt100.d \
./CODE/yawAngle.d 


# Each subdirectory must supply rules for building sources it contributes
CODE/%.src: ../CODE/%.c CODE/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fD:/Code/Embedded/SmartCar/GoAhead2022/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

CODE/%.o: ./CODE/%.src CODE/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


