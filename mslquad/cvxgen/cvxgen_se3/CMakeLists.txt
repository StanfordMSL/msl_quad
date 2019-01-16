cmake_minimum_required(VERSION 2.8.3)
project(cvxgen_se3)

add_library(cvxgen_se3
    ldl.cpp
    matrix_support.cpp
    solver.cpp
)

target_link_libraries(cvxgen_se3 m)