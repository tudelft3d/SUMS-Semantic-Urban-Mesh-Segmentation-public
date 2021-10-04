cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


set(${PROJECT_NAME}_HEADERS
    ./box.h
    ./constant.h
    ./dialogs.h
    ./file.h
	./file_dialog.h
	./IO.h
    ./kdtree.h
    ./line_stream.h
    ./mat.h
    ./point_cloud.h
    ./point_cloud_io.h
    ./properties.h
    ./quat.h
    ./resources.h
    ./string.h
    ./surface_mesh.h
    ./surface_mesh_io.h
    ./transform.h
    ./types.h
    ./vec.h
    )

set(${PROJECT_NAME}_SOURCES
    ./dialogs.cpp
	./file_dialog.cpp
    ./file.cpp
	./IO.cpp
    ./IO_obj.cpp
    ./IO_off.cpp
    ./IO_poly.cpp
    ./IO_stl.cpp
    ./kdtree.cpp
    ./ply_reader_writer.cpp
    ./point_cloud_io.cpp
    ./point_cloud_io_bin.cpp
    ./point_cloud_io_ply.cpp
    ./point_cloud_io_xyz.cpp
    ./point_cloud.cpp
    ./resources.cpp
    ./string.cpp
    ./surface_mesh.cpp
    ./surface_mesh_io.cpp
    ./surface_mesh_io_obj.cpp
    ./surface_mesh_io_off.cpp
    ./surface_mesh_io_ply.cpp
    ./transform.cpp
    )

	

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "3rd_party")


target_include_directories(${PROJECT_NAME} PRIVATE ${SemanticUrbanMesh_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${SemanticUrbanMesh_EXTERNAL})


target_compile_definitions(${PROJECT_NAME} PRIVATE GLEW_STATIC)
if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

find_package(OpenGL REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} rply tinyfiledialogs)