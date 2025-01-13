cmake_minimum_required(VERSION 3.10)

# Core
add_library(Core INTERFACE)
target_include_directories(Core INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/arm-core/CMSIS/Include")

add_library(ARM::Core ALIAS Core)

# Device
list(APPEND DEV_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/device/Source/Templates/system_stm32f4xx.c")
add_library(Device STATIC ${DEV_SOURCES})
target_include_directories(Device INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/device/Include")
target_include_directories(Device PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/device/Include")
target_link_libraries(Device PUBLIC ARM::Core)

add_library(ST::Device ALIAS Device)

# HAL/LL
list(APPEND LL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/hal/Src/stm32f4xx_ll_utils.c")
add_library(LL STATIC ${LL_SOURCES})
target_include_directories(LL INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/hal/Inc")
target_include_directories(LL PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/hal/Inc")
target_link_libraries(LL PUBLIC ST::Device)

add_library(ST::LL ALIAS LL)