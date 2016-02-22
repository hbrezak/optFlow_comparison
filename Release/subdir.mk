################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../calcErrorMetrics.cpp \
../main.cpp \
../optFlow_opencv.cpp \
../optFlow_paparazzi.cpp \
../readGroundTruth.cpp \
../read_dir_filenames.cpp \
../rgb2yuv422.cpp \
../showFlow.cpp 

C_SRCS += \
../fast_rosten.c \
../image.c \
../lucas_kanade.c 

OBJS += \
./calcErrorMetrics.o \
./fast_rosten.o \
./image.o \
./lucas_kanade.o \
./main.o \
./optFlow_opencv.o \
./optFlow_paparazzi.o \
./readGroundTruth.o \
./read_dir_filenames.o \
./rgb2yuv422.o \
./showFlow.o 

C_DEPS += \
./fast_rosten.d \
./image.d \
./lucas_kanade.d 

CPP_DEPS += \
./calcErrorMetrics.d \
./main.d \
./optFlow_opencv.d \
./optFlow_paparazzi.d \
./readGroundTruth.d \
./read_dir_filenames.d \
./rgb2yuv422.d \
./showFlow.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O3 -Wall -c -fmessage-length=0 -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


