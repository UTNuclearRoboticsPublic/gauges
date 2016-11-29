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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs
#  DEPENDS system_lib
)

set (my_plugin_cpp_SRCS
  src/rqt_gauges/my_plugin.cpp
  src/rqt_gauges/qcgaugewidget.cpp
)

set(my_plugin_cpp_HDRS
  include/rqt_gauges/my_plugin.h
  include/rqt_gauges/qcgaugewidget.h
)

set(my_plugin_cpp_UIS
  src/rqt_gauges/my_plugin.ui
)

qt4_wrap_cpp(my_plugin_cpp_MOCS ${my_plugin_cpp_HDRS})
qt4_wrap_ui(my_plugin_cpp_UIS_H ${my_plugin_cpp_UIS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/rqt_gauges
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

## The library that defines the Qt stuff
add_library(qcgaugewidget SHARED src/rqt_gauges/qcgaugewidget.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(qcgaugewidget
  ${catkin_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
    ${my_plugin_cpp_SRCS}
    ${my_plugin_cpp_MOCS}
    ${my_plugin_cpp_UIS_H}
    )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  qcgaugewidget
  ${catkin_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)
