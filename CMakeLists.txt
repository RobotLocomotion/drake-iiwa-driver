cmake_minimum_required(VERSION 2.8.5)
project(drake-kuka-driver)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}" CACHE STRING
    "Prefix for installation of sub-packages (note: required during build!)" FORCE)
endif()

message(STATUS CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

set(COMMON_CMAKE_ARGS
  CMAKE_GENERATOR ${CMAKE_GENERATOR}
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE})

set(cmake_CONFIGURE_COMMAND "true")
set(cmake_BUILD_COMMAND "true")
set(cmake_INSTALL_COMMAND "true")
# We'll manually build the subset of drake we want...
set(drake_CONFIGURE_COMMAND "true")
set(drake_BUILD_COMMAND "true")
set(drake_INSTALL_COMMAND "true")
set(kuka-fri_CONFIGURE_COMMAND "true")
set(kuka-fri_BUILD_COMMAND make -C build/GNUMake)
set(kuka-fri_INSTALL_COMMAND "true")
set(EXTERNAL_PROJECTS cmake drake kuka-fri lcm)

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
  foreach(c CONFIGURE BUILD INSTALL)
    if(DEFINED ${proj}_${c}_COMMAND)
      set(maybe_${c}_COMMAND ${c}_COMMAND ${${proj}_${c}_COMMAND})
    else()
      set(maybe_${c}_COMMAND "")
    endif()
  endforeach()
  ExternalProject_Add(${proj}
    SOURCE_DIR ${${proj}_SOURCE_DIR}
    BINARY_DIR ${${proj}_BINARY_DIR}
    DOWNLOAD_COMMAND ${${proj}_DOWNLOAD_COMMAND}
    DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    UPDATE_COMMAND ""
    ${COMMON_CMAKE_ARGS}
    ${maybe_CONFIGURE_COMMAND}
    ${maybe_BUILD_COMMAND}
    ${maybe_INSTALL_COMMAND}
    )
endforeach()

# Add the actual driver as an external project which depends on the
# others.
ExternalProject_Add(kuka-driver
  SOURCE_DIR "${PROJECT_SOURCE_DIR}/kuka-driver"
  DOWNLOAD_COMMAND ""
  UPDATE_COMMAND ""
  INSTALL_COMMAND cp bin/kuka_driver ${PROJECT_BINARY_DIR}
  DEPENDS ${EXTERNAL_PROJECTS}
  )
