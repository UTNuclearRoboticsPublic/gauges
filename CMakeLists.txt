cmake_minimum_required(VERSION 2.8.3)
project(rqt_gauges)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include ${rqt_gauges_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME} qcgaugewidget
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp std_msgs
#  DEPENDS system_lib
)

####################
## Lists of Qt files
####################

set (rqt_gauges_SRCS
  src/rqt_gauges/my_plugin.cpp
)

# Don't need this plugin's header to export
# the generic qcgaugewidget library
set(rqt_qcgauge_HDRS
  include/rqt_gauges/qcgaugewidget.h
)

# For compiling our plugin, only need the my_plugin header,
# since we'll link to `qcgaugewidget`
set(rqt_gauges_HDRS
  include/rqt_gauges/my_plugin.h
)

set(rqt_gauges_UIS
  src/rqt_gauges/my_plugin.ui
)

set(rqt_gauges_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

##############
## Qt wrapping
##############


if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
  qt5_wrap_cpp(rqt_qcgauge_MOCS ${rqt_qcgauge_HDRS})
  qt5_wrap_cpp(rqt_gauges_MOCS ${rqt_gauges_HDRS})
  qt5_wrap_ui(rqt_gauges_UIS_H ${rqt_gauges_UIS})
  set(qt_LIBRARIES Qt5::Widgets)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
  qt4_wrap_cpp(rqt_qcgauge_MOCS ${rqt_qcgauge_HDRS})
  qt4_wrap_cpp(rqt_gauges_MOCS ${rqt_gauges_HDRS})
  qt4_wrap_ui(rqt_gauges_UIS_H ${rqt_gauges_UIS})
  set(qt_LIBRARIES ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
  ${rqt_gauges_INCLUDE_DIRECTORIES}
)

## The library that defines the Qt stuff
add_library(qcgaugewidget SHARED
  src/rqt_gauges/qcgaugewidget.cpp
  ${rqt_qcgauge_MOCS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(qcgaugewidget
  ${catkin_LIBRARIES}
  ${qt_LIBRARIES}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
    ${rqt_gauges_SRCS}
    ${rqt_gauges_MOCS}
    ${rqt_gauges_UIS_H}
    )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  qcgaugewidget
  ${catkin_LIBRARIES}
  ${qt_LIBRARIES}
)

#############
## Install ##
#############

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(PROGRAMS scripts/${PROJECT_NAME}/gauge_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
