cmake_minimum_required(VERSION 3.16)
project(signalk-mini VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(ArduinoJson GIT_REPOSITORY https://github.com/bblanchon/ArduinoJson.git GIT_TAG v7.4.3)
FetchContent_MakeAvailable(ArduinoJson)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBEVENT REQUIRED libevent)

find_path(LIBCONFIG_INCLUDE_DIR libconfig.h++)
find_library(LIBCONFIGPP_LIBRARY config++)
if(NOT LIBCONFIG_INCLUDE_DIR OR NOT LIBCONFIGPP_LIBRARY)
  message(FATAL_ERROR "libconfig++ is required for Linux daemon config loading")
endif()

add_library(signalk_mini_core INTERFACE)
target_include_directories(signalk_mini_core INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/async-event-loop/src
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/data-model/src
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/nmea0183/src
  ${LIBEVENT_INCLUDE_DIRS}
)
target_compile_features(signalk_mini_core INTERFACE cxx_std_17)
target_link_directories(signalk_mini_core INTERFACE ${LIBEVENT_LIBRARY_DIRS})
target_link_libraries(signalk_mini_core INTERFACE ArduinoJson ${LIBEVENT_LIBRARIES})
target_compile_options(signalk_mini_core INTERFACE ${LIBEVENT_CFLAGS_OTHER})

add_executable(signalk-mini linux/main.cpp)
target_include_directories(signalk-mini PRIVATE ${LIBCONFIG_INCLUDE_DIR})
target_link_libraries(signalk-mini PRIVATE signalk_mini_core ${LIBCONFIGPP_LIBRARY})

include(CTest)
if(BUILD_TESTING)
  add_executable(test_signalk_mini_smoke tests/test_signalk_mini_smoke.cpp)
  target_link_libraries(test_signalk_mini_smoke PRIVATE signalk_mini_core)
  add_test(NAME test_signalk_mini_smoke COMMAND test_signalk_mini_smoke)
endif()
