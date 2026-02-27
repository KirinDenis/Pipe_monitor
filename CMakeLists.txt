cmake_minimum_required(VERSION 3.16)

# This MUST come before project() — loads all ESP-IDF toolchain magic
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(pipe_monitor)
