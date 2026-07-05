cmake_minimum_required(VERSION 3.16)
project(signalk-mini VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(ArduinoJson GIT_REPOSITORY https://github.com/bblanchon/ArduinoJson.git GIT_TAG v7.4.3)
FetchContent_MakeAvailable(ArduinoJson)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(LIBEVENT REQUIRED libevent)
pkg_check_modules(LIBCONFIG REQUIRED libconfig)

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
target_include_directories(signalk-mini PRIVATE ${LIBCONFIG_INCLUDE_DIRS})
target_link_directories(signalk-mini PRIVATE ${LIBCONFIG_LIBRARY_DIRS})
target_compile_options(signalk-mini PRIVATE ${LIBCONFIG_CFLAGS_OTHER})
target_link_libraries(signalk-mini PRIVATE signalk_mini_core ${LIBCONFIG_LIBRARIES})

include(CTest)
if(BUILD_TESTING)
  function(add_signalk_mini_test target source)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE signalk_mini_core)
    add_test(NAME ${target} COMMAND ${target})
  endfunction()

  add_signalk_mini_test(test_signalk_mini_smoke tests/test_signalk_mini_smoke.cpp)
  add_signalk_mini_test(test_nmea_tokenizer_examples tests/test_nmea_tokenizer_examples.cpp)
  add_signalk_mini_test(test_nmea_tail_sentences tests/test_nmea_tail_sentences.cpp)
  add_signalk_mini_test(test_nmea_gnss_v4 tests/test_nmea_gnss_v4.cpp)
  add_signalk_mini_test(test_nmea_multipart tests/test_nmea_multipart.cpp)
  add_signalk_mini_test(test_nmea_dsc tests/test_nmea_dsc.cpp)
  add_signalk_mini_test(test_nmea_dsc_lifecycle tests/test_nmea_dsc_lifecycle.cpp)
  add_signalk_mini_test(test_nmea_dsc_interpretation tests/test_nmea_dsc_interpretation.cpp)
  add_signalk_mini_test(test_nmea_dsc_ack tests/test_nmea_dsc_ack.cpp)
  add_signalk_mini_test(test_nmea_dsc_dse_payload tests/test_nmea_dsc_dse_payload.cpp)
  add_signalk_mini_test(test_nmea_inmarsat tests/test_nmea_inmarsat.cpp)
  add_signalk_mini_test(test_nmea_ais tests/test_nmea_ais.cpp)
  add_signalk_mini_test(test_nmea_ais_interleaved tests/test_nmea_ais_interleaved.cpp)
  add_signalk_mini_test(test_nmea_ais_own_vessel tests/test_nmea_ais_own_vessel.cpp)
  add_signalk_mini_test(test_nmea_ais_enhancements tests/test_nmea_ais_enhancements.cpp)
  add_signalk_mini_test(test_nmea_navtex tests/test_nmea_navtex.cpp)
  add_signalk_mini_test(test_nmea_navtex_interleaved tests/test_nmea_navtex_interleaved.cpp)

  add_executable(test_signalk_mini_tcp_integration tests/test_signalk_mini_tcp_integration.cpp)
  target_link_libraries(test_signalk_mini_tcp_integration PRIVATE signalk_mini_core Threads::Threads)
  add_test(NAME test_signalk_mini_tcp_integration COMMAND test_signalk_mini_tcp_integration)
endif()
