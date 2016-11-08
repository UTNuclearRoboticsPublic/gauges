cmake_minimum_required(VERSION 2.8.3)
project(gauges)

set(GCC_COVERAGE_COMPILE_FLAGS "-g -Wno-enum-compare -std=c++11 -Wenum-compare -Wformat-security")
set(GCC_COVERAGE_LINK_FLAGS    "-Wformat-security")

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

# Use C++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
  roscpp
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} graphics)

## Generate messages in the 'msg' folder
#add_message_files( FILES )

#add_service_files(
#    FILES
#)

catkin_package(
  INCLUDE_DIRS graphics
  CATKIN_DEPENDS roscpp
  DEPENDS Qt4
)

include(${QT_USE_FILE})

link_directories(${catkin_LIBRARY_DIRS})

add_executable(gauges src/gauges.cpp)
#add_dependencies(gauges)
target_link_libraries(gauges ${catkin_LIBRARIES} ${QT_LIBRARIES} )

