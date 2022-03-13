cmake_minimum_required(VERSION 3.17)
project(read_lvx)

set(CMAKE_CXX_STANDARD 14)

add_executable(read_lvx main.cpp ldq.cpp lds.cpp lds_lvx.cpp lvx_file.cpp)
target_link_libraries(read_lvx pthread)