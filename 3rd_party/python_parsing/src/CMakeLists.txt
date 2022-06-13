cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

##############################
### Find required packages ###
##############################
find_package(PythonLibs)
find_package(PythonInterp)

include_directories(${PROJECT_NAME} PRIVATE ${SemanticUrbanMesh_INCLUDE_DIR})
find_package(Boost ${Boost_VERSION})
if (${Boost_MINOR_VERSION} LESS 69)
    message(FATAL_ERROR "The current version of the Boost library is below 1.69")
else()
    set(PYTHONVERSION ${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
    find_package(Boost 1.69.0 COMPONENTS numpy graph REQUIRED)
endif()
include(FindNumPy)
include_directories(${SemanticUrbanMesh_EXTERNAL}/boost_numpy)
link_directories(${SemanticUrbanMesh_EXTERNAL}/boost_numpy/lib)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})



message("PYTHON LIBRARIES ARE " ${PYTHON_LIBRARIES})
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS} ${PYTHON_NUMPY_INCLUDE_DIR} ${SemanticUrbanMesh_EXTERNAL})
LINK_DIRECTORIES(${PYTHON_LIBRARY_DIRS})

file(GLOB python_parsing_HEADERS include/*.h)

set(CMAKE_LD_FLAG "${CMAKE_LD_FLAGS} -shared -Wl -fPIC --export-dynamic -o -O3 -Wall -std=c++11")
#message(${Boost_LIBRARIES})
add_library(python_parsing SHARED python_parsing.cpp ${python_parsing_HEADERS})
target_link_libraries(python_parsing ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} easy3d)