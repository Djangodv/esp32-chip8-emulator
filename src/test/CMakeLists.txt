# This is the project CMakeLists.txt file for the test subproject
cmake_minimum_required(VERSION 3.16)

# Include the components directory of the main application:
#
set(EXTRA_COMPONENT_DIRS "../components")

# Set the components to include the tests for.
# This can be overridden from CMake cache:
# - when invoking CMake directly: cmake -D TEST_COMPONENTS="xxxxx" ..
#
set(TEST_COMPONENTS interpreter)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(chip-8 emulator_test)

littlefs_create_partition_image(storage ../roms FLASH_IN_PROJECT)
