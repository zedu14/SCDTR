cmake_minimum_required(VERSION 3.13)
project(controlo_distribuido)

set(CMAKE_CXX_STANDARD 14)

add_executable(controlo_distribuido main.cpp controlo.cpp controlo.h node.cpp node.h)