cmake_minimum_required(VERSION 3.0.2)
project(celex5_ros_toolss)
set(CMAKE_CXX_STANDARD 11)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        cv_bridge
        celex5_msgs
        std_msgs
        )
find_package(OpenCV REQUIRED)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES celex_ros_tools
        #  CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/celex_ros_tools.cpp
# )


set(CeleX ${CMAKE_SOURCE_DIR}/celex5_ros_toolss/SDK)
set(CeleX_LIB CeleX)

include_directories(${CeleX}/include)
link_directories(${CeleX}/lib/Linux/x64)


include_directories(
        ${CeleX}/include
)
link_directories(
        ${CeleX}/lib/Linux/x64
)

# for clion
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/../devel/include)
include_directories(${CMAKE_SOURCE_DIR}/celex5_ros_toolss/SDK/include)
#for clion end

add_executable(${PROJECT_NAME}_get_event_vector
        src/celex5_getdata_ros/node.cpp
        src/celex5_getdata_ros/get_event_vector_callback.cpp)
target_link_libraries(${PROJECT_NAME}_get_event_vector
        ${catkin_LIBRARIES}
        ${CeleX_LIB})


add_executable(celex_readBin src/celex_readBin_ros.cpp)
target_link_libraries(
        celex_readBin
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CeleX_LIB}
)

add_executable(event_vector_show src/celex5_getdata_ros/get_event_vector_show.cpp)
target_link_libraries(
        event_vector_show
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)
