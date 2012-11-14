cmake_minimum_required(VERSION 2.6.0)



# build libfovis
#add_subdirectory(${PROJECT_SOURCE_DIR}/odometry/libfovis)
#set (FovisLib ${FovisLib} fovis)

#find_package(fovis REQUIRED )
find_package(EIGEN REQUIRED eigen3)

include_directories(${PROJECT_SOURCE_DIR} ${EIGEN_INCLUDE_DIRS})

# build the main library
add_library(VisualNav SHARED
    graph/graph.cpp
    planning/planning.cpp
    odometry/odometry.cpp
    utilities/kinect_interface.cpp
    )

#add_dependencies(VisualNav fovis)

add_executable(KinectMapperNav
  navigation.cpp
)

target_link_libraries(KinectMapperNav VisualNav)
