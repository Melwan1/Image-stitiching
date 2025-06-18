cmake_minimum_required(VERSION 3.28)
project(tifo)

set(CMAKE_CXX_STANDARD 20)

add_library(libtifo SHARED)
set_target_properties(libtifo PROPERTIES OUTPUT_NAME tifo)
target_include_directories(libtifo PUBLIC ${CMAKE_SOURCE_DIR}/include/libtifo)
target_compile_options(libtifo PRIVATE -Wall -Wextra -Werror -pedantic -fno-inline -g -O0)

add_executable(main)
target_link_libraries(main PUBLIC libtifo)
target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/include/libtifo)
target_compile_options(main PRIVATE -Wall -Wextra -Werror -pedantic -fno-inline -g -O0)

add_subdirectory(src)