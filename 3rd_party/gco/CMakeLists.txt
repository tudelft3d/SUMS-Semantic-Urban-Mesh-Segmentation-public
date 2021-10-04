cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

set(${PROJECT_NAME}_HEADERS
  ./block.h
  ./energy.h
  ./GCoptimization.h
  ./graph.h
  ./LinkedBlockList.h
    )

set(${PROJECT_NAME}_SOURCES
  ./GCoptimization.cpp
  ./graph.cpp
  ./LinkedBlockList.cpp
  ./maxflow.cpp
    )

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

set_target_properties(${PROJECT_NAME} PROPERTIES
                      FOLDER "3rd_party")


target_include_directories(${PROJECT_NAME} PRIVATE ${SegmentGeneration_gco_INCLUDE_DIR})

