# Find LCM
find_package(drake REQUIRED)
find_package(lcm REQUIRED)
find_package(gflags REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

set(roscpp_DIR "/opt/ros/kinetic/share/roscpp/cmake")
set(message_generation_DIR "/opt/ros/kinetic/share/message_generation/cmake")
set(sensor_msgs_DIR "/opt/ros/kinetic/share/sensor_msgs/cmake")

# Find ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

# add custom commands to copy the kuka-fri source code over to the
# build dir and compile it
set(fri_client_source ${CMAKE_SOURCE_DIR}/kuka-fri)
set(fri_client_build ${CMAKE_BINARY_DIR}/kuka-fri-build)
set(fri_client_makefile ${fri_client_build}/build/GNUMake/Makefile)
set(fri_client_library ${fri_client_build}/lib/libFRIClient.a)

# copy kuka-fri source
add_custom_command(OUTPUT ${fri_client_makefile}
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${fri_client_source} ${fri_client_build}
  COMMENT "copying FRIClient source code to build dir")

# compile kuka-fri source
add_custom_command(OUTPUT ${fri_client_library}
  COMMAND make
  WORKING_DIRECTORY ${fri_client_build}/build/GNUMake
  DEPENDS ${fri_client_makefile}
  COMMENT "building FRIClient")

# add custom target for FRIClient library
add_custom_target(fri-client DEPENDS ${fri_client_library})

# add include dir for FRIClient headers
include_directories(${fri_client_build}/include)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

# build the lcm driver that uses FRIClient
add_definitions(-std=c++11)
set(driver_name kuka_driver)
add_executable(${driver_name} kuka_driver.cc)
add_dependencies(${driver_name} fri-client)
target_link_libraries(${driver_name} ${fri_client_library} ${LCM_NAMESPACE}lcm ${gflags_LIBRARIES} drake::drake-lcmtypes-cpp ${catkin_LIBRARIES})

# install the lcm driver
install(TARGETS ${driver_name}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
