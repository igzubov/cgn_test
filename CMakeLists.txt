cmake_minimum_required(VERSION 3.12)
project(cgn_vrep)

set(CMAKE_CXX_STANDARD 14)
set(V-REP_PATH /home/igor/cgn/V-REP_PRO_EDU_V3_6_1_Ubuntu16_04)


link_directories(${V-REP_PATH})

set(SOURCE_FILES
        ${V-REP_PATH}/programming/b0RemoteApiBindings/cpp/b0RemoteApi.cpp
        ackermannCar.cpp
        pid.cpp
        pathFollower.cpp
        trajVisualizer.cpp
        main.cpp
        )

add_executable(cgn_vrep ${SOURCE_FILES})
include_directories(${V-REP_PATH}/programming/b0RemoteApiBindings/cpp/msgpack-c/include)
include_directories(${V-REP_PATH}/programming/bluezero/bindings)
include_directories(${V-REP_PATH}/programming/bluezero/include/b0/bindings)
include_directories(${V-REP_PATH}/programming/b0RemoteApiBindings/cpp/)
#target_sources(cgn_vrep PUBLIC ${V-REP_PATH}/programming/b0RemoteApiBindings/cpp/b0RemoteApi.cpp)
#target_sources(cgn_vrep PUBLIC ackermann_car.cpp)
target_link_libraries(cgn_vrep b0)
