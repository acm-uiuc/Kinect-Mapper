cmake_minimum_required(VERSION 2.6.0)

project(KINECT_MAPPER)

find_package(EIGEN REQUIRED eigen3)

set(FOVIS_DIR ${PROJECT_SOURCE_DIR}/libfovis)
add_subdirectory(${FOVIS_DIR})

set(KINECT_MAPPER_COMPONENTS utilities
                             navigation
                             graph
			     odometry
			     planning
			     quadtree
			     )

foreach(DIR ${KINECT_MAPPER_COMPONENTS})
  set(KINECT_MAPPER_SRC_DIRS ${KINECT_MAPPER_SRC_DIRS} 
                             ${PROJECT_SOURCE_DIR}/${DIR})
endforeach()

include_directories(${KINECT_MAPPER_SRC_DIRS}
                    ${EIGEN_INCLUDE_DIRS}
		    ${FOVIS_DIR}/libfovis)
set(KIN_LIBS fovis)
set(DOC_SOURCE_DIR "${PROJECT_SOURCE_DIR}")
foreach(KIN_LIB_NAME ${KINECT_MAPPER_COMPONENTS})
  add_subdirectory(${PROJECT_SOURCE_DIR}/${KIN_LIB_NAME})
  set(KIN_LIBS ${KIN_LIBS} kinect_mapper_${KIN_LIB_NAME})
  set(DOC_SOURCE_DIR "${DOC_SOURCE_DIR} ${DIR}")
endforeach()

add_executable(kinect_mapper main.cpp)
target_link_libraries(kinect_mapper ${KIN_LIBS})

find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOC_SOURCE_DIR ${KINECT_MAPPER_SRC_DIRS})
  set(doxyfile "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in" ${doxyfile})
  add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${doxyfile})
endif(DOXYGEN_FOUND)
