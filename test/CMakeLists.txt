set(LIB_NAME ${KIN_LIB_NAME}) 
set(LINK_LIBS ${KIN_LIBS})

set(LIB_SRC quadtree_testbench.cpp)

add_executable(kinect_mapper_${LIB_NAME} ${LIB_SRC})
target_link_libraries(kinect_mapper_${LIB_NAME} ${LINK_LIBS} ${Boost_LIBRARIES})

