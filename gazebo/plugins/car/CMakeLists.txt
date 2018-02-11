cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

#manual ICE
FIND_PATH( Ice_INCLUDE_DIR NAMES Ice/Ice.h  PATHS ENV C++LIB ENV)

IF( Ice_INCLUDE_DIR )
    FIND_LIBRARY( Ice_LIBRARY1 NAMES Ice PATHS ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64 )
    FIND_LIBRARY( Ice_LIBRARY2 NAMES IceUtil PATHS ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
    SET (Ice_LIBRARIES ${Ice_LIBRARY1} ${Ice_LIBRARY2})
    IF( Ice_LIBRARIES )
                MESSAGE ("-- Ice found at ${Ice_LIBRARIES}")
    ENDIF( Ice_LIBRARIES )
ENDIF(Ice_INCLUDE_DIR)

IF(NOT Ice_LIBRARIES)
                MESSAGE ("*** Ice not found")
ENDIF()

set(JDE_DIR /home/andresjhe/JdeRobot)
#set(JDE_LIBS ${JDE_DIR}/src/stable/libs)
#set(INTERFACES_CPP_DIR ${JDE_DIR}/src/stable/interfaces/cpp/jderobot
                        #${JDE_DIR}/src/stable/interfaces/cpp/)
set(INTERFACES_CPP_DIR /usr/local/include/jderobot)
set(LIBS_NEEDED  colorspaces jderobotutil progeo pioneer fuzzylib visionlib parallelIce )


include_directories(${CMAKE_SOURCE_DIR}
                    ${JDE_LIBS}
                    ${INTERFACES_CPP_DIR}
                    ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS}
                ${INTERFACES_CPP_DIR})
FOREACH(LibFile ${LIBS_NEEDED})
    link_directories(${LIBS_DIR}/${LibFile})
    MESSAGE("${LIBS_DIR}/${LibFile}")
ENDFOREACH(LibFile)

add_library(carMotors SHARED carMotors.cc)

target_link_libraries(carMotors
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Ice_LIBRARIES}
    #colorspacesmm
    #JderobotInterfaces.so
    )
