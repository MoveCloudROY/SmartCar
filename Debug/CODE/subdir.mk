################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CODE/ImageDeal.c \
../CODE/ImagePreDeal.c \
../CODE/motor.c \
../CODE/pid.c \
../CODE/stack.c \
../CODE/vofa.c 

OBJS += \
./CODE/ImageDeal.o \
./CODE/ImagePreDeal.o \
./CODE/motor.o \
./CODE/pid.o \
./CODE/stack.o \
./CODE/vofa.o 

COMPILED_SRCS += \
./CODE/ImageDeal.src \
./CODE/ImagePreDeal.src \
./CODE/motor.src \
./CODE/pid.src \
./CODE/stack.src \
./CODE/vofa.src 

C_DEPS += \
./CODE/ImageDeal.d \
./CODE/ImagePreDeal.d \
./CODE/motor.d \
./CODE/pid.d \
./CODE/stack.d \
./CODE/vofa.d 


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


