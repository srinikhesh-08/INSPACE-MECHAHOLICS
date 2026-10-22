# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)
set(EXTRA_COMPONENT_DIRS "../../FSW/Rocketry-firmware/components/fsw_radio")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gcs_bridge)
