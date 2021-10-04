cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH})

set(${PROJECT_NAME}_HEADERS
	./feature_computation.hpp
	./math_base.hpp
	./mesh_io.hpp
	./operation_function.hpp
	./parameters.hpp
	./property_parsing.hpp
	./PTCloud.hpp
	./random_generator.hpp
	./sampling_function.hpp
	./SFMesh.hpp
	./space_iterators.hpp
	./spatial_hashing.hpp
	./super_segment.hpp
	./mesh_classifier.hpp
	./over_segmentation.hpp
	./CGAL_Define.hpp
	./mesh_processor.hpp
    )

set(${PROJECT_NAME}_SOURCES
    ./main.cpp
	./feature_computation.cpp
	./mesh_io.cpp
	./operation_function.cpp
	./parameters.cpp
	./property_parsing.cpp
	./sampling_function.cpp
	./mesh_classifier.cpp
	./over_segmentation.cpp
	./mesh_processor.cpp
    )

include_directories(${PROJECT_NAME} PRIVATE ${SemanticUrbanMesh_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PythonLibs REQUIRED)

set(boost_linked_libraries)
set(boost_compile_definitions)
find_package(Boost COMPONENTS thread system filesystem serialization iostreams bzip2 zlib)
if (Boost_SERIALIZATION_FOUND AND Boost_IOSTREAMS_FOUND AND Boost_ZLIB_FOUND)
	set(_Boost_BZIP2_HEADERS             "boost/iostreams/filter/bzip2.hpp")
	set(_Boost_ZLIB_HEADERS              "boost/iostreams/filter/zlib.h")
	set(boost_linked_libraries ${boost_linked_libraries}
		${Boost_SYSTEM_LIBRARY}
		${Boost_FILESYSTEM_LIBRARY}
		${Boost_SERIALIZATION_LIBRARY}
		${Boost_IOSTREAMS_LIBRARY}
		${Boost_ZLIB_LIBRARY} 
		${Boost_BZIP2_LIBRARY})
	set(boost_compile_definitions ${boost_compile_definitions}
		"-DCGAL_LINKED_WITH_BOOST_IOSTREAMS"
		"-DCGAL_LINKED_WITH_BOOST_SERIALIZATION")
else()
	message(STATUS "NOTICE: Boost Serialization or IO Streams or ZLIB not found. Classification plugin won't be available.")
endif()

set(classification_linked_libraries)
set(classification_compile_definitions)

if (TBB_FOUND)
  message(STATUS "Found TBB ${TBB_VERSION}")
  set(classification_linked_libraries ${classification_linked_libraries}
	${TBB_LIBRARIES}
	${TBB_MALLOC_LIBRARY}
	${TBB_MALLOC_PROXY_LIBRARY})
  set(classification_compile_definitions ${classification_compile_definitions}
	"-DCGAL_LINKED_WITH_TBB")
  include_directories( ${TBB_INCLUDE_DIR} )
else()
  message(STATUS "NOTICE: TBB not found, parallel for classification won't be available.")
endif()

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS} ${SemanticUrbanMesh_EXTERNAL})

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS}) #create win32 application

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${boost_linked_libraries} ${PYTHON_LIBRARIES} ${classification_linked_libraries} gco easy3d Eigen3::Eigen CGAL::CGAL)
target_compile_definitions(${PROJECT_NAME} PUBLIC ${classification_compile_definitions} ${boost_compile_definitions} WIN32_LEAN_AND_MEAN NOMINMAX -DCGAL_NO_PRECONDITIONS -DHAVE_SNPRINTF)
