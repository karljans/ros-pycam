cmake_minimum_required(VERSION 3.0.0)
project(py_camera)

set(LIB_MAJOR_VERSION "0")
set(LIB_MINOR_VERSION "0")
set(LIB_PATCH_VERSION "1")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

find_package(catkin REQUIRED COMPONENTS)

catkin_package(CATKIN_DEPENDS)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS src/pycam.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
