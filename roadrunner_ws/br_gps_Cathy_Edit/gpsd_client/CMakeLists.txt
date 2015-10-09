cmake_minimum_required(VERSION 2.8.3)
project(gpsd_client)

############
## Catkin ##
############

find_package(catkin REQUIRED
  roscpp
  gps_common
  sensor_msgs
)

find_package(PkgConfig)
pkg_check_modules (libgps REQUIRED libgps)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package()

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${libgps_INCLUDE_DIRS})
add_executable(gpsd src/client.cpp)
target_link_libraries(gpsd ${catkin_LIBRARIES} ${libgps_LIBRARIES})

#############
## Install ##
#############

install(TARGETS gpsd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
