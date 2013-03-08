cmake_minimum_required(VERSION 2.6.0)

project(KINECT_MAPPER)

find_package(EIGEN REQUIRED eigen3)
find_package(Boost COMPONENTS unit_test_framework REQUIRED)

find_path(OPENNI_INCLUDE_DIRS
    NAMES XnOS.h
    PATH_SUFFIXES ni)
if("${OPENNI_INCLUDE_DIRS}" STREQUAL "OPENNI_INCLUDE_DIRS-NOTFOUND")
    set(OPENNI_EXAMPLE_MESSAGE "Disabled - XnOS.h not found")
else()
    find_library(OPENNI_LIBS OpenNI)
    if("${OPENNI_LIBS}" STREQUAL "OPENNI_LIBS-NOTFOUND")
        set(OPENNI_EXAMPLE_MESSAGE "Disabled - OpenNI library not found")
    else()
        set(OPENNI_EXAMPLE_MESSAGE "Enabled")
    endif()
endif()

find_path(LIBFREENECT_INCLUDE_DIRS
    NAMES libfreenect.h
    PATH_SUFFIXES libfreenect)
if("${LIBFREENECT_INCLUDE_DIRS}" STREQUAL "LIBFREENECT_INCLUDE_DIRS-NOTFOUND")
    set(LIBFREENECT_EXAMPLE_MESSAGE "Disabled - libfreenect.h not found")
else()
    find_library(LIBFREENECT_LIBS freenect HINTS /usr/local/lib64)
    if("${LIBFREENECT_LIBS}" STREQUAL "LIBFREENECT_LIBS-NOTFOUND")
        set(LIBFREENECT_EXAMPLE_MESSAGE "Disabled - libfreenect not found")
    else()
        set(LIBFREENECT_EXAMPLE_MESSAGE "Enabled")
    endif()
endif()

include_directories(${OPENNI_INCLUDE_DIRS})

set(FOVIS_DIR ${PROJECT_SOURCE_DIR}/libfovis)
add_subdirectory(${FOVIS_DIR})

set(KINECT_MAPPER_COMPONENTS utilities
                             graph
			     odometry
			     planning
			     controls
			     #quadtree
			     )

foreach(DIR ${KINECT_MAPPER_COMPONENTS})
  set(KINECT_MAPPER_SRC_DIRS ${KINECT_MAPPER_SRC_DIRS} 
                             ${PROJECT_SOURCE_DIR}/${DIR})
endforeach()

include_directories(${KINECT_MAPPER_SRC_DIRS}
                    ${EIGEN_INCLUDE_DIRS}
		    ${FOVIS_DIR}/libfovis
		    ${PROJECT_SOURCE_DIR}/quadtree
		    )
set(KIN_LIBS fovis ${OPENNI_LIBS} ${LIBFREENECT_LIBS})
set(DOC_SOURCE_DIR "${PROJECT_SOURCE_DIR}")
foreach(KIN_LIB_NAME ${KINECT_MAPPER_COMPONENTS})
  add_subdirectory(${PROJECT_SOURCE_DIR}/${KIN_LIB_NAME})
  set(KIN_LIBS ${KIN_LIBS} kinect_mapper_${KIN_LIB_NAME})
  set(DOC_SOURCE_DIR "${DOC_SOURCE_DIR} ${DIR}")
endforeach()

add_executable(kinect_mapper main.cpp)
target_link_libraries(kinect_mapper ${KIN_LIBS})
set(KIN_LIB_NAME quadtree_test)
add_subdirectory(${PROJECT_SOURCE_DIR}/test)

find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOC_SOURCE_DIR ${KINECT_MAPPER_SRC_DIRS})
  set(doxyfile "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in" ${doxyfile})
  add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${doxyfile})
endif(DOXYGEN_FOUND)
