
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Bluepad32 library location
set(BLUEPAD32_ROOT "${CMAKE_CURRENT_LIST_DIR}/../bluepad32")

# BTstack configuration
set(ENV{BTSTACK_ROOT} "${BLUEPAD32_ROOT}/external/btstack")

# External ESP-IDF components
set(EXTRA_COMPONENT_DIRS "${BLUEPAD32_ROOT}/src/components")

# ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# BTstack integration
include($ENV{BTSTACK_ROOT}/port/esp32/components/btstack/cmake/BTstack_project.cmake)

project(pingpong-robot)
