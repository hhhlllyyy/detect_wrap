cmake_minimum_required(VERSION 3.4)

project(detect_wrap)
find_package(PkgConfig)

pkg_search_module(OPENCV REQUIRED opencv)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O2 -g")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include_")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/kit")

set(CORE_FILE lsd.c rotationmath.cpp VPDetection.cpp wrapImageRef.cpp)

#add_library( )
#target_link_libraries(imo_rectify ${OpenCV_LIBS})

add_executable(MAIN  main.cpp ${CORE_FILE} )
target_link_libraries(MAIN  ${OPENCV_LIBRARIES} curl)



