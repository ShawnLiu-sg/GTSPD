cmake_minimum_required(VERSION 3.15)
project(same_stop_truck_drone LANGUAGES CXX)

add_executable(truck_drone main.cpp search.cpp model.cpp exact.cpp)
target_compile_features(truck_drone PRIVATE cxx_std_17)
set_target_properties(truck_drone PROPERTIES CXX_EXTENSIONS OFF)
