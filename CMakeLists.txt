cmake_minimum_required(VERSION 3.21)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/toolchain.cmake" CACHE PATH "" FORCE)
project(blinky LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(MCU cortex-m4)
set(LDSCRIPT ${CMAKE_SOURCE_DIR}/linker/memory.ld)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(fw.elf
    src/main.c
    src/spi.c
    src/protocol.c
    src/encoder.c
    src/ringBuffer.c
    src/control.c
    src/drive.c
    src/loops.c
    src/clock.c
    src/tim1.c
    src/pwm.c
    src/drv8353.c
    src/system_stm32f4xx.c
    src/current_feedback.c
    src/sysid/foc_sysid.c
    src/sysid/foc_trajectory.c
    startup/startup_stm32f411xe.s
)

target_include_directories(fw.elf PRIVATE
    ${CMAKE_SOURCE_DIR}/Include
    ${CMAKE_SOURCE_DIR}/Include/sysid
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)

target_compile_definitions(fw.elf PRIVATE
    STM32F411xE
)

target_compile_options(fw.elf PRIVATE
    $<$<COMPILE_LANGUAGE:C>:
        -mcpu=${MCU}
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -ffreestanding
        -fno-builtin
        -fdata-sections
        -ffunction-sections
        -Og
        -g3
        -Wall
        -Wextra
    >
    $<$<COMPILE_LANGUAGE:ASM>:
        -mcpu=${MCU}
        -mthumb
        -x assembler-with-cpp
    >
)

target_link_options(fw.elf PRIVATE
    -mcpu=${MCU}
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -T${LDSCRIPT}
    -nostartfiles
    -specs=nano.specs
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/fw.map
    -L${ARM_TOOLCHAIN_ROOT}/arm-none-eabi/lib/thumb/v7e-m+fp/hard
    -Wl,--start-group
        -lc
        -lgcc
        -lm
        -lnosys
    -Wl,--end-group
)

target_link_libraries(fw.elf PRIVATE gcc)
target_link_libraries(fw.elf PRIVATE m)

set_property(TARGET fw.elf APPEND PROPERTY LINK_DEPENDS ${LDSCRIPT})

add_custom_command(TARGET fw.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:fw.elf> ${CMAKE_BINARY_DIR}/fw.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:fw.elf> ${CMAKE_BINARY_DIR}/fw.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:fw.elf>
)