cmake_minimum_required(VERSION 2.8.3)
project(rqt_gauges)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rqt_gui
  rqt_gui_cpp
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Qt4 COMPONENTS QtCore QtGUi REQUIRED)

include(${QT_USE_FILE})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

roslint_cpp()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs
#  DEPENDS system_lib
)

set (mainwindow_cpp_SRCS
  src/mainwindow.cpp
  src/qcgaugewidget.cpp
)

set(mainwindow_cpp_HDRS
  include/mainwindow.h
  include/qcgaugewidget.h
)

set(mainwindow_cpp_UIS
  resources/mainwindow.ui
)

qt4_wrap_cpp(mainwindow_cpp_MOCS ${mainwindow_cpp_HDRS})
qt4_wrap_ui(mainwindow_cpp_UIS_H ${mainwindow_cpp_UIS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
    ${mainwindow_cpp_SRCS}
    ${mainwindow_cpp_MOCS}
    ${mainwindow_cpp_UIS_H}
    )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)
