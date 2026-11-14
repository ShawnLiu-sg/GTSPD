cmake_minimum_required(VERSION 3.15)
project(truck_drone_comparison LANGUAGES CXX)

add_executable(truck_drone_compare compare.cpp io.cpp search.cpp model.cpp exact.cpp order_first.cpp vns.cpp)
target_compile_features(truck_drone_compare PRIVATE cxx_std_17)
set_target_properties(truck_drone_compare PROPERTIES CXX_EXTENSIONS OFF)
