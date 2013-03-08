set(LIB_NAME ${KIN_LIB_NAME}) 
set(LINK_LIBS ${KIN_LIBS})

set(LIB_SRC aux_socket.cpp robot.cpp server.cpp)

add_library(kinect_mapper_${LIB_NAME} ${LIB_SRC})
target_link_libraries(kinect_mapper_${LIB_NAME} ${LINK_LIBS})

#add_executable(server.out server.cpp)
#target_link_libraries(server.out kinect_mapper_${LIB_NAME} ${LINK_LIBS} pthread)

add_executable(client.out client.cpp)
target_link_libraries(client.out kinect_mapper_${LIB_NAME} ${LINK_LIBS})
