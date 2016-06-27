cmake_minimum_required(VERSION 2.8.5)
project(drake-kuka-driver)

set(cmake_BUILD_COMMANND "")

# We'll manually build the subset of drake we want...
set(drake_CONFIGURE_COMMANND "")
set(drake_BUILD_COMMANND "")
set(kuka-fri_BUILD_COMMAND make -C build/GNUMake)
set(EXTERNAL_PROJECTS drake kuka-fri)

include(ExternalProject)
find_package(Git REQUIRED)

foreach(proj ${EXTERNAL_PROJECTS})
  set(${proj}_SOURCE_DIR "${PROJECT_SOURCE_DIR}/externals/${proj}")

  # Compute the path to the submodule for this external.
  file(RELATIVE_PATH ${proj}_GIT_SUBMODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${${proj}_SOURCE_DIR})

  # Download externals as Git submodules.
  set(${proj}_DOWNLOAD_COMMAND ${GIT_EXECUTABLE} submodule update --init -- ${${proj}_GIT_SUBMODULE_PATH})

  # Initialize the submodule configuration now so parallel downloads do not conflict later.
  execute_process(COMMAND ${GIT_EXECUTABLE} submodule init -- ${${proj}_GIT_SUBMODULE_PATH}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  if(NOT ${proj}_BINARY_DIR)
    # In-source build for non-CMake projects.
    set(${proj}_BINARY_DIR "${${proj}_SOURCE_DIR}")
  endif()
  ExternalProject_Add(${proj}
    SOURCE_DIR ${${proj}_SOURCE_DIR}
    BINARY_DIR ${${proj}_BINARY_DIR}
    DOWNLOAD_COMMAND ${${proj}_DOWNLOAD_COMMAND}
    DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND "${${proj}_BUILD_COMMAND}"
    INSTALL_COMMAND ""
    )
endforeach()
