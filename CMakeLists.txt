cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

################################################################################

set (CMAKE_CXX_STANDARD 11)

################################################################################

# Detects whether this is a top-level project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	set(SemanticUrbanMesh_TOPLEVEL_PROJECT ON)
else()
	set(SemanticUrbanMesh_TOPLEVEL_PROJECT OFF)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

################################################################################

### Configuration
set(SemanticUrbanMesh_ROOT "${CMAKE_CURRENT_LIST_DIR}")
set(SemanticUrbanMesh_INCLUDE_DIR "${SemanticUrbanMesh_ROOT}")
set(SemanticUrbanMesh_SOURCE_DIR "${SemanticUrbanMesh_ROOT}")
set(SemanticUrbanMesh_EXTERNAL "${SemanticUrbanMesh_ROOT}/3rd_party")

set(SegmentClassification_gco_INCLUDE_DIR "${SegmentClassification_EXTERNAL}/gco")
set(SemanticUrbanMesh_rply_INCLUDE_DIR "${SemanticUrbanMesh_EXTERNAL}/rply")
### conditionally compile certain modules depending on libraries found on the system
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

################################################################################

add_subdirectory(3rd_party)
add_subdirectory(semantic_urban_mesh_segmentation)
