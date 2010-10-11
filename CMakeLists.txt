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

# standard
find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})

# dashel
find_path(DASHEL_INCLUDE_DIR dashel.h
	/usr/include/dashel
	/usr/local/include/dashel
	REQUIRED
)
find_library(DASHEL_LIBRARY dashel
	/usr/lib
	/usr/local/lib
	REQUIRED
)
include_directories(${DASHEL_INCLUDE_DIR})
set(EXTRA_LIBS ${DASHEL_LIBRARY})
if (WIN32)
	set(EXTRA_LIBS ${EXTRA_LIBS} winspool ws2_32)
else (WIN32)
	if (APPLE)
		find_library(COREFOUNDATION_LIBRARY CoreFoundation REQUIRED)
		find_library(IOKIT_LIBRARY IOKit REQUIRED)
		mark_as_advanced(COREFOUNDATION_LIBRARY)
		mark_as_advanced(IOKIT_LIBRARY)
		set(EXTRA_LIBS ${EXTRA_LIBS} ${COREFOUNDATION_LIBRARY} ${IOKIT_LIBRARY} ${EXTRA_LIBS})
	else (APPLE)
		find_package(PkgConfig)
		pkg_check_modules(HAL hal)
		include_directories(${HAL_INCLUDE_DIRS})
		set(EXTRA_LIBS ${EXTRA_LIBS} ${HAL_LIBRARIES} ${EXTRA_LIBS})
	endif (APPLE)
endif (WIN32)

# aseba
find_path(ASEBA_INCLUDE_DIR msg/msg.h
	/usr/include/aseba
	/usr/local/include/aseba
	REQUIRED
)
find_library(ASEBA_CORE_LIBRARY asebacore
	/usr/lib/
	/usr/local/lib/
	REQUIRED
)
find_library(ASEBA_COMPILER_LIBRARY asebacompiler
	/usr/lib/
	/usr/local/lib/
	REQUIRED
)
include_directories(${ASEBA_INCLUDE_DIR})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(asebaros src/asebaros.cpp)
target_link_libraries(asebaros ${ASEBA_COMPILER_LIBRARY} ${ASEBA_CORE_LIBRARY} ${DASHEL_LIBRARY} ${LIBXML2_LIBRARIES} ${EXTRA_LIBS})
