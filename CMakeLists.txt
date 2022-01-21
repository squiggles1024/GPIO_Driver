#Build with cmake -DCMAKE_MAKE_PROGRAM=make.exe -G "Unix Makefiles" ..
cmake_minimum_required(VERSION 3.16.3)                                                          #@application specific: Installed version of cmake
set(CMAKE_VERBOSE_MAKEFILE ON)                                                                  #Enable verbose make file
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/toolchain.cmake)                                   #Directory of toolchain file

###############################################################################
# NAME / PROJECT
###############################################################################
project(main)                                                                                   #@application specific: project name
set(EXECUTABLE ${PROJECT_NAME}.elf)                                                             #Output executable name
set(LINKER_FILE ${CMAKE_SOURCE_DIR}/drivers/hardware/STM32F411CEUX_FLASH.ld)                             #@application specific:Linker script file

###############################################################################
# LANGUAGE CONFIG
###############################################################################
enable_language(C ASM)                                                                          #Enable C and ASM Languages                                                                        
set(CMAKE_C_STANDARD 99)                                                                        #Set C standard to C99
set(CMAKE_C_STANDARD_REQUIRED ON)                                                               #Initializes C Standard Required on all targets
set(CMAKE_C_EXTENSIONS OFF)                                                                     #Initializes C Extensions Required on all targets

# Optional: issue a message to be sure it uses the correct toolchain file.
message(STATUS "CMAKE_TOOLCHAIN_FILE is: ${CMAKE_TOOLCHAIN_FILE}")


###############################################################################
# SOURCE
###############################################################################
set(APP_SRC_DIR "application/")
set(DRIVERS_DIR "drivers/")
set(MIDDLEWARES_DIR "middlewares/")
set(HAL_DIR "HAL/")
set(BSP_DIR "board_support/")

set(CMSIS_DIR ${HAL_DIR}CMSIS/)
set(CONFIG_DIR ${DRIVERS_DIR}configuration/)
set(HARDWARE_DIR ${DRIVERS_DIR}hardware/)

set(SRC_FILES
        ${HARDWARE_DIR}startup_stm32f411ceux.s
        ${HARDWARE_DIR}system_stm32f4xx.c
        ${APP_SRC_DIR}main.c
        ${APP_SRC_DIR}irq.c
        ${DRIVERS_DIR}gpio.c
        ${DRIVERS_DIR}common.c
        ${CONFIG_DIR}gpio_config.c 
        )

# Build the executable based on the source files
add_executable(${EXECUTABLE} ${SRC_FILES})

###############################################################################
# DEFINES
###############################################################################
target_compile_definitions(${EXECUTABLE} PRIVATE
    -DDEBUG 
    -DSTM32F411xE                                                                                #@application Specific
    #-DUSE_HAL_DRIVER                                                                            #@application Specific
    )

###############################################################################
# INCLUDES
###############################################################################
target_include_directories(${EXECUTABLE} PRIVATE
        ${APP_DIR}
        ${DRIVERS_DIR}
        ${MIDDLEWARES_DIR}
        ${HAL_DIR}
        ${BSP_DIR}
        ${CMSIS_DIR}
        ${CONFIG_DIR}
        ${HARDWARE_DIR}
        )

###############################################################################
# C FLAGS
###############################################################################
target_compile_options(${EXECUTABLE} PRIVATE
    -mcpu=cortex-m4        #Cortex M4 CPU
    -mthumb                #Thumb instructions
    -mfloat-abi=hard       #Floating point instructions using hardware instructions
    -mfpu=fpv4-sp-d16      #Cortex M4F processors include the FPv4-SP-D16 floating point extension
    -g3                    #Debug level 3 (max)
    -c                     #Compile into a .o file, do not link yet.
    -O0                    #No optimization
    -ffunction-sections    #Generate a seperate ELF section for each function in src files
    -fdata-sections        #Generate a seperate ELF section for each variable in src file
    -fstack-usage          #Generate a .su (stack usage) file for each source
    --specs=nano.specs     #Compile using newlib c
    -Wall                  #Show all compiler warnings
    )

###############################################################################
# L FLAGS
###############################################################################
target_link_options(${EXECUTABLE} PRIVATE
        -mcpu=cortex-m4                            #Cortex M4 CPU
        -T${LINKER_FILE}                           #Specify linker script
        --specs=nosys.specs                        #System calls should be implemented as stubs
        -Wl,-Map=${PROJECT_NAME}.map,--cref        #Generate a map file with symbol cross reference table
        -Wl,--gc-sections                          #Discard unused sections
        -static --specs=nano.specs                 #Prevent linking with dynamic libraries.  Link with newlib C
        -mfpu=fpv4-sp-d16                          #Cortex M4F processors include the FPv4-SP-D16 floating point extension
        -mfloat-abi=hard                           #Floating point instructions using hardware instructions
        -mthumb                                    #Thumb instructions
        -Wl,--start-group 
        -lc                                        #Libraries between start group and end group will repeatedly be searched until no unresolved symbols remain
        -lm 
        -Wl,--end-group
        )

#Print executable size at the end
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD COMMAND ${CMAKE_SIZE_UTIL} ${EXECUTABLE})

# Create Hex, Bin, and S files
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD COMMAND ${CMAKE_OBJCOPY} -O srec --srec-len=64 ${EXECUTABLE} ${PROJECT_NAME}.s19 COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)