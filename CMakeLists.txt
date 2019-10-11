# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesPost-Processing)

    add_executable(rs-post-processing rs-post-processing.cpp ./third-party/imgui/example.hpp ./third-party/imgui.cpp ./third-party/imgui_draw.cpp ./third-party/imgui_impl_glfw.cpp ./librealsense2/rs.hpp)
    set_property(TARGET rs-post-processing PROPERTY CXX_STANDARD 11)
    target_link_libraries(rs-post-processing ${DEPENDENCIES})
    include_directories(rs-post-processing third-party/ third-party/imgui librealsense2)
    set_target_properties (rs-post-processing PROPERTIES FOLDER "Examples")
    install(TARGETS rs-post-processing RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
