cmake_minimum_required(VERSION 3.15)

project(STM32_test)

enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)


# build config
add_compile_options(
  -DSTM32F411xE
  -DUSE_FULL_LL_DRIVER
)

# flag
add_compile_options(
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
  -fdata-sections
  -ffunction-sections
  -Wall
  $<$<CONFIG:Debug>:-Og>
)

add_link_options(
  -T${CMAKE_SOURCE_DIR}/core/FLASH.ld
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
  -specs=nano.specs
  -lc
  -lm
  -lnosys
  -Wl,-Map=${PROJECT_NAME}.map,--cref
  -Wl,--gc-sections
)

add_subdirectory(core)

set(TARGET_NAME ${PROJECT_NAME})

add_executable(${TARGET_NAME} 
  src/main.c
  "core/device/Source/Templates/gcc/startup_stm32f411xe.s"
)
target_link_libraries(${TARGET_NAME} ST::LL)

# Print executable size
add_custom_command(TARGET ${TARGET_NAME}
  POST_BUILD
  COMMAND arm-none-eabi-size ${TARGET_NAME}
)

# Create hex file
add_custom_command(TARGET ${TARGET_NAME}
  POST_BUILD
  COMMAND arm-none-eabi-objcopy -O ihex ${TARGET_NAME} ${TARGET_NAME}.hex
  COMMAND arm-none-eabi-objcopy -O binary ${TARGET_NAME} ${TARGET_NAME}.bin
)

install(TARGETS ${TARGET_NAME} ${TARGET_NAME}.hex ${TARGET_NAME}.bin ${TARGET_NAME}.map
  DESTINATION bin
)
