cmake_minimum_required(VERSION 3.14.7)

project(motor_test)

add_library(common
  arduino/motor_test/common.h
  arduino/motor_test/common.cpp)

target_include_directories(common
  PUBLIC
    arduino)

function(add_tool name)
  set(target tool_${name})
  add_executable(${target} ${ARGN})
  target_link_libraries(${target} PUBLIC common)
  set_target_properties(${target}
    PROPERTIES
      OUTPUT_NAME ${name})
endfunction()

#add_tool(readport read.cpp)
#add_tool(to_csv to_csv.cpp)
#add_tool(check check.cpp)
add_tool(sim sim.cpp capture.h capture.cpp)
