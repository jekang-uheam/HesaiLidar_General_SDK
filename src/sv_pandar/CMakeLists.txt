CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
project(SVPandarSDK)

set(CMAKE_CXX_FLAGS "-fPIC --std=c++17")

include_directories(
  .
  include
)

add_library(${PROJECT_NAME}
  src/sv_pandar_64ch.cpp
  src/sv_pandar_128ch.cpp
  src/sv_pandar_core_sdk.cpp
)