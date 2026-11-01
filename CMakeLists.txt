cmake_minimum_required(VERSION 3.14)
project(diffbot CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT MSVC)
  add_compile_options(-Wall -Wextra -O2)
endif()

add_library(diffbot INTERFACE)
target_include_directories(diffbot INTERFACE include)

add_executable(diffbot_sim sim/simulate.cpp)
target_link_libraries(diffbot_sim PRIVATE diffbot)

enable_testing()
add_executable(diffbot_tests tests/test_main.cpp)
target_link_libraries(diffbot_tests PRIVATE diffbot)
add_test(NAME unit COMMAND diffbot_tests)
add_test(NAME sim_tracking COMMAND diffbot_sim)
