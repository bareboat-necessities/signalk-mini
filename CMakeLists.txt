cmake_minimum_required(VERSION 3.16)
project(signalk-mini VERSION 0.1.0 LANGUAGES CXX)

option(SIGNALK_MINI_FETCH_ARDUINOJSON "Fetch ArduinoJson v7 for protocol boundary" ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(signalk_mini_core INTERFACE)
target_include_directories(signalk_mini_core INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/async-event-loop/src
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/data-model/src
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/nmea0183/src
)
target_compile_features(signalk_mini_core INTERFACE cxx_std_11)

if(SIGNALK_MINI_FETCH_ARDUINOJSON)
  include(FetchContent)
  FetchContent_Declare(
    ArduinoJson
    GIT_REPOSITORY https://github.com/bblanchon/ArduinoJson.git
    GIT_TAG v7.4.3
  )
  FetchContent_MakeAvailable(ArduinoJson)
  target_link_libraries(signalk_mini_core INTERFACE ArduinoJson)
endif()

add_executable(signalk-mini linux/main.cpp)
target_link_libraries(signalk-mini PRIVATE signalk_mini_core)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_signalk_mini_smoke tests/test_signalk_mini_smoke.cpp)
  target_link_libraries(test_signalk_mini_smoke PRIVATE signalk_mini_core)
  add_test(NAME test_signalk_mini_smoke COMMAND test_signalk_mini_smoke)
endif()
