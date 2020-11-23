project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp)

target_link_libraries (uWS LINK_PUBLIC ${OPENSSL_CRYPTO_LIBRARY})
target_link_libraries (uWS LINK_PUBLIC ${ZLIB_LIBRARY})
target_link_libraries (uWS LINK_PUBLIC pthread)

install (TARGETS uWS DESTINATION /usr/lib64)
install (FILES src/Extensions.h src/WebSocketProtocol.h src/Networking.h src/WebSocket.h src/Hub.h src/Group.h src/Node.h src/Socket.h src/HTTPSocket.h src/uWS.h src/uUV.h DESTINATION /usr/include/uWS)


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
