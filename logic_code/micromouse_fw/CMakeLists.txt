cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME micromouse_fw)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C CXX ASM)
set(CMAKE_CXX_STANDARD 17)

add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/../core/maze.cpp
    ${CMAKE_SOURCE_DIR}/../core/planner.cpp
    ${CMAKE_SOURCE_DIR}/../core/moves.cpp
    ${CMAKE_SOURCE_DIR}/../core/motion.cpp
    ${CMAKE_SOURCE_DIR}/../core/mouse.cpp
    ${CMAKE_SOURCE_DIR}/../stm32/port_stm32.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/../stm32
    ${CMAKE_SOURCE_DIR}/../core
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

)
