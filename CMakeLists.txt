cmake_minimum_required(VERSION 3.16)
project(signalk-mini VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SIGNALK_MINI_FETCH_ARDUINOJSON "Fetch ArduinoJson v7 for protocol boundary" ON)
option(SIGNALK_MINI_BUILD_MODULE_TESTS "Build imported module tests" ON)

add_library(pypilot_data_model INTERFACE)
target_include_directories(pypilot_data_model INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/modules/data-model/src)
target_compile_features(pypilot_data_model INTERFACE cxx_std_11)

add_library(pypilot_nmea0183_connector INTERFACE)
target_include_directories(pypilot_nmea0183_connector INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/nmea0183/src
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/data-model/src
)
target_compile_features(pypilot_nmea0183_connector INTERFACE cxx_std_11)

add_library(async_event_loop INTERFACE)
target_include_directories(async_event_loop INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/modules/async-event-loop/src)
target_compile_features(async_event_loop INTERFACE cxx_std_17)
find_path(SIGNALK_MINI_LIBEVENT_INCLUDE_DIR event2/event.h)
if(SIGNALK_MINI_LIBEVENT_INCLUDE_DIR)
  target_include_directories(async_event_loop INTERFACE ${SIGNALK_MINI_LIBEVENT_INCLUDE_DIR})
endif()

add_library(signalk_mini_core INTERFACE)
target_include_directories(signalk_mini_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(signalk_mini_core INTERFACE pypilot_data_model pypilot_nmea0183_connector async_event_loop)

if(SIGNALK_MINI_FETCH_ARDUINOJSON)
  include(FetchContent)
  FetchContent_Declare(ArduinoJson GIT_REPOSITORY https://github.com/bblanchon/ArduinoJson.git GIT_TAG v7.4.3)
  FetchContent_MakeAvailable(ArduinoJson)
  target_link_libraries(signalk_mini_core INTERFACE ArduinoJson)
endif()

add_executable(signalk-mini linux/main.cpp)
target_link_libraries(signalk-mini PRIVATE signalk_mini_core)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_signalk_mini_stage tests/test_signalk_mini_stage.cpp)
  target_link_libraries(test_signalk_mini_stage PRIVATE signalk_mini_core)
  add_test(NAME test_signalk_mini_stage COMMAND test_signalk_mini_stage)

  if(SIGNALK_MINI_BUILD_MODULE_TESTS)
    foreach(test_name
        test_model
        test_field_map
        test_extended_model
        test_compat_name_coverage)
      add_executable(data_model_${test_name} modules/data-model/tests/${test_name}.cpp)
      target_link_libraries(data_model_${test_name} PRIVATE pypilot_data_model)
      target_include_directories(data_model_${test_name} PRIVATE modules/data-model/tests/support)
      add_test(NAME data_model_${test_name} COMMAND data_model_${test_name})
    endforeach()

    foreach(test_name
        test_checksum
        test_stream_parser
        test_nmea0183_helpers
        test_apply_gps
        test_apply_wind
        test_apply_heading
        test_apply_water_lwy
        test_apply_apb
        test_apply_source
        test_format_sentence
        test_format_pypilot_outputs
        test_first_class_inputs)
      add_executable(nmea0183_${test_name} modules/nmea0183/tests/${test_name}.cpp)
      target_link_libraries(nmea0183_${test_name} PRIVATE pypilot_nmea0183_connector)
      add_test(NAME nmea0183_${test_name} COMMAND nmea0183_${test_name})
    endforeach()

    if(SIGNALK_MINI_LIBEVENT_INCLUDE_DIR)
      add_executable(async_event_loop_test_memory_stream modules/async-event-loop/tests/test_memory_stream.cpp)
      target_link_libraries(async_event_loop_test_memory_stream PRIVATE async_event_loop)
      add_test(NAME async_event_loop_test_memory_stream COMMAND async_event_loop_test_memory_stream)

      add_executable(async_event_loop_test_protocol_readers modules/async-event-loop/tests/test_protocol_readers.cpp)
      target_link_libraries(async_event_loop_test_protocol_readers PRIVATE async_event_loop)
      add_test(NAME async_event_loop_test_protocol_readers COMMAND async_event_loop_test_protocol_readers)
    else()
      message(STATUS "libevent headers not found; async-event-loop compile tests skipped")
    endif()
  endif()
endif()
