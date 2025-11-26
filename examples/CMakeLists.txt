cmake_minimum_required(VERSION 3.10)

# Example executables
add_executable(basic_example basic_example.cpp)
target_link_libraries(basic_example CADLib)

add_executable(curves_example curves_example.cpp)
target_link_libraries(curves_example CADLib)

add_executable(surfaces_example surfaces_example.cpp)
target_link_libraries(surfaces_example CADLib)

add_executable(transformations_example transformations_example.cpp)
target_link_libraries(transformations_example CADLib)
