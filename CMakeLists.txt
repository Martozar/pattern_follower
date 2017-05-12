cmake_minimum_required(VERSION 2.8)
project(pattern_follower)

add_definitions(-std=c++14)
file(GLOB SOURCES src/*)
file(GLOB HEADERS include/*)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O4 -march=native -Wfatal-errors -pthread -g")

find_package(OpenCV 3.1.0 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})

option(WITH_LASER "Using Hokuyo rangefinder" ON)

if (WITH_LASER)
  find_package(HokuyoAIST)
  include_directories(${Flexiport_INCLUDE_DIRS})
  include_directories(${HokuyoAIST_INCLUDE_DIRS})
  add_definitions(-DWITH_LASER)
  add_definitions(${HokuyoAIST_DEFINITIONS})
  target_link_libraries(${PROJECT_NAME} ${HokuyoAIST_LIBRARIES})
endif()

add_executable(${PROJECT_NAME}-main main.cpp)
set_target_properties(${PROJECT_NAME}-main PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_link_libraries( ${PROJECT_NAME}-main ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(TARGETS ${PROJECT_NAME}-main DESTINATION bin)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
