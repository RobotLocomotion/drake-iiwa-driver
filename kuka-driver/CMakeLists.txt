cmake_minimum_required(VERSION 2.8.5)
project(kuka-driver)

include (../externals/cmake/lcmtypes.cmake)
include (../externals/cmake/pods.cmake)
add_lcmtype(../externals/drake/drake/lcmtypes/lcmt_iiwa_command.lcm)
add_lcmtype(../externals/drake/drake/lcmtypes/lcmt_iiwa_status.lcm)
lcmtypes_build()

include_directories(../externals/kuka-fri/include)
add_executable(kuka_driver kuka_driver.cc)
target_link_libraries(kuka_driver "-L${PROJECT_SOURCE_DIR}/../externals/kuka-fri/lib"
  "-lFRIClient" "-lpopt")
add_dependencies(kuka_driver kuka_driver_lcmtypes)
pods_use_pkg_config_packages(kuka_driver lcm)
