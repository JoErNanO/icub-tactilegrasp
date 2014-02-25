# Copyright: 2009 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Andrea Del Prete, Alexander Schmitz, Francesco Giovannini
# email:   andrea.delprete@iit.it, alexander.schmitz@iit.it, francesco.giovannini@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

cmake_minimum_required(VERSION 2.6)

set(PROJECTNAME tactileGrasp)
project(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)

#optionally: use cmake find scripts provided by YARP and iCub
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_INSTALL_PREFIX "$ENV{ICUB_DIR}" CACHE PATH "Install path prefix, pre-appended onto install directory." FORCE)

# load support files
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${ICUBCONTRIB_MODULE_PATH})
include(iCubHelpers)
include(YarpInstallationHelpers)
include(ICUBcontribOptions)
include(ICUBcontribHelpers)
#icubcontrib_set_default_prefix()

# Add the example application "tactileGraspingDemo"
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Path where to install the applications")
add_subdirectory(tactileGraspingDemo)
icub_app_all()

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_Directories(${skinDynLib_INCLUDE_DIRS})

# Search for source code.
set(INC_SOURCES
    src/GazeThread.cpp
    src/GraspThread.cpp
    src/TactileGrasp.cpp
    src/TactileGraspMain.cpp
)
set(INC_HEADERS
    include/iCub/tactileGrasp/GazeThread.h
    include/iCub/tactileGrasp/GraspThread.h
    include/iCub/tactileGrasp/TactileGrasp.h
)
source_group("Source Files" FILES ${INC_SOURCES})
source_group("Header Files" FILES ${INC_HEADERS})
add_executable(${PROJECTNAME}  ${INC_SOURCES} ${INC_HEADERS})

target_link_libraries(${PROJECTNAME}  ${YARP_LIBRARIES} skinDynLib icubmod)

if(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin/Release)
else(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif(WIN32)
#yarp_install(FILES GurlsClassificationRecursive.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})


icubcontrib_add_uninstall_target()
