# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(FetchContent)

# Nombre lógico del firmware (portable)
get_filename_component(FW_NAME ${CMAKE_SOURCE_DIR} NAME CACHE STRING "Firmware name")

project(${FW_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${FW_NAME})

add_subdirectory(lib/drivers)
add_subdirectory(lib/utils)
add_subdirectory(src)

FetchContent_Declare(
    kitronik_pico_robotics
    GIT_REPOSITORY https://github.com/axfer-io/kitronik-pico-robotics.git
    GIT_TAG main
)

FetchContent_MakeAvailable(kitronik_pico_robotics)

target_link_libraries(${FW_NAME} PRIVATE
    pico_stdlib
    drivers
    utils
    kitronik_pico_robotics
)

pico_enable_stdio_usb(${FW_NAME} 1)
pico_enable_stdio_uart(${FW_NAME} 0)

pico_add_extra_outputs(${FW_NAME})

# --- Flash (Debug Probe via OpenOCD) ---
set(PICO_FLASH_SCRIPT "$ENV{HOME}/pico/tools/flash.sh")

if(EXISTS "${PICO_FLASH_SCRIPT}")
    add_custom_target(flash
        COMMAND ${PICO_FLASH_SCRIPT} $<TARGET_FILE:${FW_NAME}>
        DEPENDS ${FW_NAME}
        USES_TERMINAL
    )
else()
    message(WARNING "flash target disabled: no flash script at ${PICO_FLASH_SCRIPT}")
endif()

