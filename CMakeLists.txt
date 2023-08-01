cmake_minimum_required(VERSION 3.1.0)

option(test "Build tests." OFF)

add_compile_options(-std=c++17)

project(ylm_ros)

find_package(catkin QUIET COMPONENTS roscpp sensor_msgs std_msgs roslib)

if (catkin_FOUND)

  find_package(Boost REQUIRED)

  ###################################
  ## catkin specific configuration ##
  ###################################

  set(YLM_INCLUDE_DIRS
    "include;")

  catkin_package(
	  INCLUDE_DIRS ${YLM_INCLUDE_DIRS}
     LIBRARIES ylm_ros
	  CATKIN_DEPENDS roscpp sensor_msgs std_msgs roslib
      DEPENDS message_runtime
  )

  include_directories(${YLM_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

  if (test)
    message(STATUS "Tests of library enabled")
    add_definitions(-D_UNIT_TESTS_)
  endif (test)
  
  add_library(ylm_client STATIC src/lumotive_client_interface.cpp src/ylm_client.cpp)
  target_link_libraries(ylm_client ${Boost_LIBRARIES})

  add_executable(lumotive_driver src/lumotive_driver.cpp src/network_handler.cpp src/lumotive_cloud_creation.cpp)
  target_link_libraries(lumotive_driver ylm_client ${catkin_LIBRARIES})
  add_dependencies(lumotive_driver ylm_client)
  
  # catkin_install_python(PROGRAMS scripts/multi_m20_files_generator.py
  #  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  ## Mark executables and/or libraries for installation
  install(TARGETS lumotive_driver
     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


if (test)

  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  find_package(catkin REQUIRED rosbag)

  add_rostest_gtest(lumotive_ros_tests tests/lumotive_ros_tests.test tests/lumotive_ros_tests.cpp)
  target_link_libraries(lumotive_ros_tests ${catkin_LIBRARIES})

endif (test)

endif (catkin_FOUND)

unset(test CACHE) # avoid variable being cached
