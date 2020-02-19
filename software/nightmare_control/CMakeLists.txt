cmake_minimum_required(VERSION 2.8.3)
project(nightmare_control)

find_package(catkin REQUIRED COMPONENTS
  python_qt_binding
  roslib
  # rospkg
  rviz
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_install_python(PROGRAMS 
  src/nightcontrol.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
