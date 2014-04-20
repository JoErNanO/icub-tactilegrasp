# Copyright: 2009 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Andrea Del Prete, Alexander Schmitz, Francesco Giovannini
# email:   andrea.delprete@iit.it, alexander.schmitz@iit.it, francesco.giovannini@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 


set(MODULENAME tactileGrasp)

# Search for source code.
set(INC_HEADERS
    idl/include/tactileGrasp_IDLServer.h
    include/iCub/tactileGrasp/GazeThread.h
    include/iCub/tactileGrasp/GraspThread.h
    include/iCub/tactileGrasp/TactileGraspModule.h
)

set(INC_SOURCES
    idl/src/tactileGrasp_IDLServer.cpp
    GazeThread.cpp
    GraspThread.cpp
    TactileGraspModule.cpp
    main.cpp
)

# Search for thrift files
set(IDL ${MODULENAME}.thrift)
yarp_idl_to_dir(${IDL} ${CMAKE_CURRENT_SOURCE_DIR}/idl)


source_group("Source Files" FILES ${INC_SOURCES})
source_group("Header Files" FILES ${INC_HEADERS})
source_group("IDL Files"    FILES ${IDL})

# Include directories
include_Directories(${skinDynLib_INCLUDE_DIRS})
include_directories(include)
include_directories(idl/include)

add_executable(${MODULENAME} ${INC_SOURCES} ${INC_HEADERS} ${IDL})
target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} skinDynLib icubmod)

if(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin/${CMAKE_BUILD_TYPE})
else(WIN32)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif(WIN32)
