cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# dependencies

# # Build dashel
# execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.dashel RESULT_VARIABLE _make_failed)
# if(_make_failed)
#   message(FATAL_ERROR "Build of Dashel failed")
# endif(_make_failed)
# 
# # Link dashel
# include_directories(${PROJECT_SOURCE_DIR}/dashel/include)
# link_directories(${PROJECT_SOURCE_DIR}/dashel/lib)
# if (WIN32)
# 	set(EXTRA_LIBS ${EXTRA_LIBS} winspool ws2_32)
# else (WIN32)
# 	if (APPLE)
# 		find_library(COREFOUNDATION_LIBRARY CoreFoundation REQUIRED)
# 		find_library(IOKIT_LIBRARY IOKit REQUIRED)
# 		mark_as_advanced(COREFOUNDATION_LIBRARY)
# 		mark_as_advanced(IOKIT_LIBRARY)
# 		set(EXTRA_LIBS ${EXTRA_LIBS} ${COREFOUNDATION_LIBRARY} ${IOKIT_LIBRARY} ${EXTRA_LIBS})
# 	else (APPLE)
# 		find_package(PkgConfig)
# 		pkg_check_modules(HAL hal)
# 		include_directories(${HAL_INCLUDE_DIRS})
# 		set(EXTRA_LIBS ${EXTRA_LIBS} ${HAL_LIBRARIES} ${EXTRA_LIBS})
# 	endif (APPLE)
# endif (WIN32)
# include_directories(${PROJECT_SOURCE_DIR}/dashel/include)
# link_directories(${PROJECT_SOURCE_DIR}/dashel/lib)
# 
# # Build aseba
# execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.aseba RESULT_VARIABLE _make_failed)
# if(_make_failed)
#   message(FATAL_ERROR "Build of Aseba failed")
# endif(_make_failed)
# 
# # Link aseba
# include_directories(${PROJECT_SOURCE_DIR}/aseba/include)
# link_directories(${PROJECT_SOURCE_DIR}/aseba/lib)

# libxml2
find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_boost_directories()
rosbuild_add_executable(asebaros src/asebaros.cpp)
rosbuild_link_boost(asebaros thread)
target_link_libraries(asebaros asebacompiler asebacore dashel ${LIBXML2_LIBRARIES} ${EXTRA_LIBS})
