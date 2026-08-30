cmake_minimum_required(VERSION 3.18)

project(chrono_capped_cylinder_benchmark LANGUAGES CXX)

find_package(Chrono REQUIRED COMPONENTS Multicore CONFIG)

add_executable(benchmark_capped_cylinder benchmark_capped_cylinder.cpp)
target_compile_features(benchmark_capped_cylinder PRIVATE cxx_std_17)
target_link_libraries(benchmark_capped_cylinder PRIVATE ${CHRONO_TARGETS})

