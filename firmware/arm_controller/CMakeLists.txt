# Arm Controller Firmware — 4-DOF Robotic Arm + Gripper
# Target: ESP32-S3  |  Framework: ESP-IDF v5.x
cmake_minimum_required(VERSION 3.16)

# Pull in the ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(arm_controller)
