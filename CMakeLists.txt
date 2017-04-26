cmake_minimum_required(VERSION 2.8)
project(pattern_follower)

add_definitions(-std=c++14)
file(GLOB SOURCES src/*)
file(GLOB HEADERS include/*)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O4 -march=native -Wfatal-errors -pthread -g")

find_package(OpenCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}-main main.cpp)
set_target_properties(${PROJECT_NAME}-main PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_link_libraries( ${PROJECT_NAME}-main ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(TARGETS ${PROJECT_NAME}-main DESTINATION bin)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
