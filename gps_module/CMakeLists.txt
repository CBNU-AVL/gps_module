cmake_minimum_required(VERSION 2.8.12)

project(gps_module)
add_compile_options(-std=c++11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include_directories(
    /usr/include
    include/
    )

link_directories(
    /usr/lib
    )
link_libraries(
    serial
    pthread
    )
add_executable(${PROJECT_NAME} "src/main.cpp" "src/parser_vercpp.cpp")

