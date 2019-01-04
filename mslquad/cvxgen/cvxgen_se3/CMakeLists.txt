cmake_minimum_required(VERSION 2.8.3)
project(cvxgen_se3)

add_library(cvxgen_se3
    ldl.c
    matrix_support.c
    solver.c
)

target_link_libraries(cvxgen_se3 m)