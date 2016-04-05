################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../DungBotEmptyController.cpp \
../DungBotSimulation.cpp \
../dungbot.cpp \
../main.cpp 

OBJS += \
./DungBotEmptyController.o \
./DungBotSimulation.o \
./dungbot.o \
./main.o 

CPP_DEPS += \
./DungBotEmptyController.d \
./DungBotSimulation.d \
./dungbot.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/home/theis/lpzrobots/ode_robots/include -I/home/theis/lpzrobots/selforg/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


