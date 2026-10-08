cmake_minimum_required(VERSION 3.16)

# 1. Pull in ESP-IDF's build logic (paths, toolchains, build rules)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 2. Name your project (this sets the final binary name: my_project.bin)
project(stormborn)
