CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
# project name
PROJECT(deltarobot)

# message output
MESSAGE(STATUS "This is the BINARY dir: " ${PROJECT_BINARY_DIR})
MESSAGE(STATUS "This is the SOURCE dir: " ${PROJECT_SOURCE_DIR})

# add compile options
add_compile_options(-std=c++11)
add_compile_options(-fPIC)
add_compile_options(-pthread)
add_compile_options(-DASIO_STANDALONE)
add_compile_options(-DASIO_HAS_STD_CHRONO)

# include directories
INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/../sdk/include
)

# link directories
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/../sdk/lib/linux_x86_64)

# set library output path
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# set the library file name
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src LIB_DELTAROBOTCONTROL_SRC)

# add library
ADD_LIBRARY(DeltaRobotControl_dynamic SHARED ${LIB_DELTAROBOTCONTROL_SRC})
ADD_LIBRARY(DeltaRobotControl_static STATIC ${LIB_DELTAROBOTCONTROL_SRC})
# set library properties
SET_TARGET_PROPERTIES(DeltaRobotControl_dynamic PROPERTIES OUTPUT_NAME "DeltaRobotControl")
SET_TARGET_PROPERTIES(DeltaRobotControl_dynamic PROPERTIES VERSION 1.1 SOVERSION 1)
SET_TARGET_PROPERTIES(DeltaRobotControl_static PROPERTIES OUTPUT_NAME "DeltaRobotControl")

# target link libraries
TARGET_LINK_LIBRARIES(DeltaRobotControl_dynamic ActuatorController pthread)
TARGET_LINK_LIBRARIES(DeltaRobotControl_static ActuatorController pthread)

