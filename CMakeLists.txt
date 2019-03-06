#	\file       CMakeLists.txt
#	\brief      for CMake to generate makefile
#	\author     Yu-Shiang Yan
#	\version    1.0.0.x
#	\date       2019/03/05

cmake_minimum_required(VERSION 3.0.2)

option(VERBOS_MAKEFILE "verbose makefile" OFF)
set(CMAKE_VERBOSE_MAKEFILE VERBOS_MAKEFILE)

project(eye-in-hand-calibration)

# manage the dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")


# add configuration types
set(CMAKE_CONFIGURATION_TYPES Debug Release)

# option for user to choose which target want to build
option(BUILD_VISION "build the vision lib" OFF)
option(BUILD_UNIT_TEST_VISION "build the uint test of vision lib" OFF)


set(WIN32_SDK "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.0A/Include")

# set the position independent code property on all targets
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# check the compiler type
if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    message("Configure for GNU toolchains")
    set(PRE_DEFINE_HEAD "-D")
    add_definitions(${PRE_DEFINE_HEAD}_USING_GNU_)
    add_definitions(-fpack-struct=16) # 16 bytes alignment
    add_definitions(-Wall) # enables all the warnings
    add_definitions(-malign-double)
#    add_definitions(
#      -Wno-unused-variable
#      -Wno-unused-function
#      -Wno-unused-but-set-variable
#      ) # disable un-use var and function warning
    #add_definitions(-Werror) # make all warnings into errors
    set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} -std=c99)
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=gnu++11)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,$ORIGIN")
    if (MINGW)
        message("MinGW32 toolchains")
        add_definitions(${PRE_DEFINE_HEAD}_USING_WIN32_)
        set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin/mingw)
        # add the Win32 APIs include path
        set(INC_PATH "${INC_PATH} ${WIN32_SDK}")
        message("Add Win32 SDK path " ${WIN32_SDK})
    elseif (CYGWIN)
        message(STATUS "cygwin")
        add_definitions(${PRE_DEFINE_HEAD}_USING_CYGWIN_)
        add_definitions(${PRE_DEFINE_HEAD}_ARCH_X86_)
        set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin/cygwin)
        set(CMAKE_SHARED_LIBRARY_PREFIX "")
    else ()
        message("Linux x86_64 platform")
        add_definitions(${PRE_DEFINE_HEAD}_USING_LINUX_)
        add_definitions(${PRE_DEFINE_HEAD}_ARCH_X86_64_)
        add_definitions(${PRE_DEFINE_HEAD}_LUA_REDIRECTION_)
        set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin/linux)
    endif ()
    option(RELEASE_BUILD "-O3, dont use this option with -g3 at the same time" OFF)
    if (RELEASE_BUILD)
        set(RELEASE_DEF "-O3")
        add_definitions("-O3")
        add_definitions(${PRE_DEFINE_HEAD}_RELEASE_)
        message("Release build with " ${RELEASE_DEF} " flags.")
    else ()
        set(DEBUG_DEF "-g3")
        add_definitions(-g3)
        add_definitions(${PRE_DEFINE_HEAD}_DEBUG_)
        set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin/linux/debug)
        message("Debug build with " ${DEBUG_DEF} " flags.")
    endif ()
elseif (MSVC)
    message("Configure for Microsoft Visual C++")
    set(PRE_DEFINE_HEAD "/D")
    add_definitions(${PRE_DEFINE_HEAD}_USING_MSVC_)
    add_definitions(/Zp16) # 16 bytes alignment
    set(CMAKE_PREFIX_PATH "C:/Qt/Qt5.7.1/5.7/msvc2015")
    message("Configure for WIN32")
    add_definitions(${PRE_DEFINE_HEAD}_USING_WIN32_)
    if (DEBUG_BUILD)
      add_definitions(/MTd)
    endif()
    set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin/msvc)
else ()
    message("unknown tool chain")
endif ()


# set include path of all target
set(INC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/inc)
include_directories(${INC_PATH})
include_directories(${INC_PATH}/Vision/interface)
include_directories(${INC_PATH}/Vision/opencv/include/opencv4)
message("Set global include path to " ${INC_PATH})

# set the version number
set (THE_VERSION_MAJOR 2)
set (THE_VERSION_MINOR 7)
configure_file (
    ${INC_PATH}/VERSION.h.in
    ${INC_PATH}/VERSION.h
)

# set output path of all target
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR})
message("Set output directory to " ${OUTPUT_DIR})

foreach (OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${OUTPUT_DIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${OUTPUT_DIR})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${OUTPUT_DIR})
endforeach (OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES)

if (BUILD_VISION)
    message("build the Vision")
    set(BUILD_TITLE Vision)
    set(BUILD_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Vision/Image_Processing.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Vision/ImageAlgo_HandEye_Calibration.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Vision/Utility.cpp
        )

    link_directories(${INC_PATH}/Vision/opencv/lib)
    link_directories(${INC_PATH}/Vision/opencv/lib/opencv4/3rdparty)

    link_directories(${BUILD_TITLE} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    add_library(${BUILD_TITLE} SHARED ${BUILD_SRCS})

    target_link_libraries(${BUILD_TITLE} opencv_core opencv_calib3d opencv_features2d opencv_flann opencv_ml opencv_highgui opencv_imgcodecs
                                         opencv_imgproc)


    add_definitions(-march=core2)
    add_definitions(-Wno-unknown-pragmas)
    target_compile_definitions(${BUILD_TITLE} PRIVATE _DLL_EXPORTS_)
    set(SHARED_LIBS ${SHARED_LIBS} ${BUILD_TITLE})
endif ()

if (BUILD_UNIT_TEST_VISION)
    message("build the unit test of Vision")
        set(BUILD_TITLE UnitTestVision)
    set(BUILD_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/UnitTest/UnitTestVision.cpp
        )

    link_directories(${BUILD_TITLE} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    add_executable(${BUILD_TITLE} ${BUILD_SRCS})

    target_link_libraries(${BUILD_TITLE} Vision pthread)

    set(EXEC_BINS ${EXEC_BINS} ${BUILD_TITLE})
endif ()


# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

INSTALL(
    TARGETS ${EXEC_BINS} ${SHARED_LIBS} ${STATIC_LIBS}
    RUNTIME DESTINATION ./
    LIBRARY DESTINATION ./
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)


