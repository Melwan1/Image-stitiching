cmake_minimum_required(VERSION 3.28)
project(tifo)

set(CMAKE_CXX_STANDARD 20)

add_library(libtifo SHARED)

include(FetchContent)
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG c9371de # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)

FetchContent_MakeAvailable(yaml-cpp)
target_link_libraries(libtifo PUBLIC yaml-cpp::yaml-cpp) # The library or executable that require yaml-cpp library

set_target_properties(libtifo PROPERTIES OUTPUT_NAME tifo)
target_include_directories(libtifo PUBLIC ${CMAKE_SOURCE_DIR}/include/libtifo)
target_compile_options(libtifo PRIVATE -Wall -Wextra -Werror -pedantic -fno-inline -g -O0)

add_executable(main)

target_link_libraries(main PUBLIC libtifo)
target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/include/libtifo)
target_compile_options(main PRIVATE -Wall -Wextra -Werror -pedantic -fno-inline -g -O0)

add_subdirectory(src)