# Copyright: 2009 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Andrea Del Prete, Alexander Schmitz, Francesco Giovannini
# email:   andrea.delprete@iit.it, alexander.schmitz@iit.it, francesco.giovannini@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

cmake_minimum_required(VERSION 2.6)

set(PROJECTNAME tactileGrasp)
project(${PROJECTNAME})


# Check build type
if(NOT CMAKE_BUILD_TYPE)
    MESSAGE(STATUS "No build type specified. Defaulting to Release.")
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
# Add flags
MESSAGE(STATUS "Compiling in ${CMAKE_BUILD_TYPE} mode.")
if(CMAKE_BUILD_TYPE MATCHES Debug)
    MESSAGE(STATUS "Adding debug flags to compiler.")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
else(CMAKE_BUILD_TYPE MATCHES Debug)
    MESSAGE(STATUS "Adding release flags to compiler.")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
endif(CMAKE_BUILD_TYPE MATCHES Debug)


# Find packages
find_package(YARP)
find_package(ICUB)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
include(YarpInstallationHelpers)

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

add_definitions(${YARP_DEFINES}) # add yarp definitions - this contains also -D_REENTRANT


# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_Directories(${skinDynLib_INCLUDE_DIRS})


# Search for source code.
set(INC_HEADERS
    include/tactileGrasp_IDLServer.h
    include/iCub/tactileGrasp/GazeThread.h
    include/iCub/tactileGrasp/GraspThread.h
    include/iCub/tactileGrasp/TactileGraspModule.h
)

set(INC_SOURCES
    src/tactileGrasp_IDLServer.cpp
    src/GazeThread.cpp
    src/GraspThread.cpp
    src/TactileGraspModule.cpp
    src/main.cpp
)

# Search for application files
set(APP_SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/app/applications/TactileGrasp.xml.template)

# Search for ini files
set(APP_CONF ${CMAKE_CURRENT_SOURCE_DIR}/app/contexts/${PROJECTNAME}/confTactileGrasp.ini)

# Search for thrift files
set(IDL ${PROJECTNAME}.thrift)

source_group("Source Files" FILES ${INC_SOURCES})
source_group("Header Files" FILES ${INC_HEADERS})
source_group("IDL Files"    FILES ${IDL})

add_executable(${PROJECTNAME} ${INC_SOURCES} ${INC_HEADERS} ${IDL})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} skinDynLib icubmod)

if(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin/${CMAKE_BUILD_TYPE})
else(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif(WIN32)

yarp_install(FILES ${APP_SCRIPTS} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${APP_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME})

icubcontrib_add_uninstall_target()
