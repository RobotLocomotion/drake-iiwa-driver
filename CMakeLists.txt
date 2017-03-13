cmake_minimum_required(VERSION 2.8.5)
project(drake-kuka-driver)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_RPATH})

if(NOT EXISTS ${CMAKE_SOURCE_DIR}/kuka-fri/.git)
  message(FATAL_ERROR "\nThe kuka-fri submodule is not available.\nPlease run: git submodule update --init")
endif()

add_subdirectory(kuka-driver)
