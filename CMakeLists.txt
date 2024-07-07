# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

#set(ENV{IDF_TARGET} "esp32c3") #设置默认开发板为C3

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(24-ESP32-INA226-Branch)