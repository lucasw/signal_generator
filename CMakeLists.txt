cmake_minimum_required(VERSION 2.8.3)
project(signal_generator)

# add_compile_options(-std=c++1z)

find_package(catkin REQUIRED COMPONENTS
  ddynamic_reconfigure
  dynamic_reconfigure
  roslint
  rospy
  std_msgs
  visualization_msgs
)

set(
    ROSLINT_CPP_OPTS
     "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references"
)

# roslint_cpp()
roslint_python()

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS
  ddynamic_reconfigure
  dynamic_reconfigure
  std_msgs
  visualization_msgs
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/integrator.py
  scripts/signal_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

