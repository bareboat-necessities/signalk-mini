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
  add_executable(test_signalk_mini_smoke tests/test_signalk_mini_smoke.cpp)
  target_link_libraries(test_signalk_mini_smoke PRIVATE signalk_mini_core)
  add_test(NAME test_signalk_mini_smoke COMMAND test_signalk_mini_smoke)

  add_executable(test_nmea_tokenizer_examples tests/test_nmea_tokenizer_examples.cpp)
  target_link_libraries(test_nmea_tokenizer_examples PRIVATE signalk_mini_core)
  add_test(NAME test_nmea_tokenizer_examples COMMAND test_nmea_tokenizer_examples)

  add_executable(test_nmea_tail_sentences tests/test_nmea_tail_sentences.cpp)
  target_link_libraries(test_nmea_tail_sentences PRIVATE signalk_mini_core)
  add_test(NAME test_nmea_tail_sentences COMMAND test_nmea_tail_sentences)

  add_executable(test_nmea_multipart tests/test_nmea_multipart.cpp)
  target_link_libraries(test_nmea_multipart PRIVATE signalk_mini_core)
  add_test(NAME test_nmea_multipart COMMAND test_nmea_multipart)

  add_executable(test_nmea_ais tests/test_nmea_ais.cpp)
  target_link_libraries(test_nmea_ais PRIVATE signalk_mini_core)
  add_test(NAME test_nmea_ais COMMAND test_nmea_ais)

  add_executable(test_nmea_ais_interleaved tests/test_nmea_ais_interleaved.cpp)
  target_link_libraries(test_nmea_ais_interleaved PRIVATE signalk_mini_core)
  add_test(NAME test_nmea_ais_interleaved COMMAND test_nmea_ais_interleaved)

  add_executable(test_nmea_ais_own_vessel tests/test_nmea_ais_own_vessel.cpp)
  target_link_libraries(test_nmea_ais_own_vessel PRIVATE signalk_mini_core)
  add_test(NAME test_nmea_ais_own_vessel COMMAND test_nmea_ais_own_vessel)

  add_executable(test_nmea_navtex tests/test_nmea_navtex.cpp)
  target_link_libraries(test_nmea_navtex PRIVATE signalk_mini_core)
  add_test(NAME test_nmea_navtex COMMAND test_nmea_navtex)

  add_executable(test_nmea_navtex_interleaved tests/test_nmea_navtex_interleaved.cpp)
  target_link_libraries(test_nmea_navtex_interleaved PRIVATE signalk_mini_core)
  add_test(NAME test_nmea_navtex_interleaved COMMAND test_nmea_navtex_interleaved)

  add_executable(test_signalk_mini_tcp_integration tests/test_signalk_mini_tcp_integration.cpp)
  target_link_libraries(test_signalk_mini_tcp_integration PRIVATE signalk_mini_core Threads::Threads)
  add_test(NAME test_signalk_mini_tcp_integration COMMAND test_signalk_mini_tcp_integration)
endif()
