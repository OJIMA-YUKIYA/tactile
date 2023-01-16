cmake_minimum_required(VERSION 2.8.3)

set(esp32_path "/dev/ttyUSB_esp32")
set(momo_path "${PROJECT_SOURCE_DIR}/serial_out")


configure_file(
    "${PROJECT_SOURCE_DIR}/serialPathConfig.h.in"
    "${PROJECT_BINARY_DIR}/serialPathConfig.h"
)

include_directories(
    ${PROJECT_BINARY_DIR}
)

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  ${Boost_INCLUDE_DIRS}
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_executable(main main.cpp)
set_target_properties(
    main
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
)


target_link_libraries(main
  ${Boost_LIBRARIES}
  ${libsegwayrmp_LIBRARIES}
)
