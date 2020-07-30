SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.7)

SET(TOOLCHIN_PATH "/opt/gcc-arm-none-eabi-9-2019-q4-major/bin")

#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.7)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)
SET(CMAKE_C_COMPILER ${TOOLCHIN_PATH}/arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER_WORKS 1)
SET(CMAKE_CXX_COMPILER ${TOOLCHIN_PATH}/arm-none-eabi-g++)
set(AS ${TOOLCHIN_PATH}/arm-none-eabi-as)
set(AR ${TOOLCHIN_PATH}/arm-none-eabi-ar)
set(OBJCOPY ${TOOLCHIN_PATH}/arm-none-eabi-objcopy)
set(OBJDUMP ${TOOLCHIN_PATH}/arm-none-eabi-objdump)
set(SIZE ${TOOLCHIN_PATH}/arm-none-eabi-size)


SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld)

#Uncomment for hardware floating point
#SET(FPU_FLAGS "-mfloat-abi=hard -mfpu=fpv4-sp-d16")
#add_definitions(-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1)

#Uncomment for software floating point
#SET(FPU_FLAGS "-mfloat-abi=soft")

SET(COMMON_FLAGS
    "-mcpu=cortex-m3 ${FPU_FLAGS} -mthumb -mthumb-interwork -ffunction-sections -fdata-sections \
    -g -fno-common -fmessage-length=0 -specs=nosys.specs -specs=nano.specs -Os")

SET(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

PROJECT(windrider C CXX ASM)
set(CMAKE_CXX_STANDARD 17)

#add_definitions(-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1)
add_definitions(-D__weak=__attribute__\(\(weak\)\) -D__packed=__attribute__\(\(__packed__\)\) -DUSE_HAL_DRIVER -DSTM32F103xB)

file(GLOB_RECURSE SOURCES "startup/*.*" 
                        "Drivers/*.*" 
                        "Src/*.*" 
                        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.*
                        Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.*)

include_directories(Inc 
                    Drivers/STM32F1xx_HAL_Driver/Inc Drivers/STM32F1xx_HAL_Driver/Inc/Legacy 
                    Drivers/CMSIS/Device/ST/STM32F1xx/Include Drivers/CMSIS/Include 
                    Drivers/STM32F1xx_HAL_Driver/Inc 
                    Drivers/STM32F1xx_HAL_Driver/Inc/Legacy 
                    Drivers/CMSIS/Device/ST/STM32F1xx/Include 
                    Drivers/CMSIS/Include
                    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
                    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")