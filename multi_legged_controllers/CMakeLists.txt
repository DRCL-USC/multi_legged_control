cmake_minimum_required(VERSION 3.10)
project(multi_legged_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        controller_interface
        legged_deployment
        )   

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        controller_interface
        legged_deployment
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/MultiLeggedController.cpp
        src/visualization/ModifiedLeggedRobotVisualizer.cpp
        src/visualization/FootPlacementVisualization.cpp
        src/visualization/SphereVisualization.cpp
        src/legged_estimation/ModifiedStateEstimateBase.cpp
        src/legged_estimation/ModifiedLinearKalmanFilter.cpp
        src/PlanarTerrainReceiver.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


add_executable(bag_file_node src/bagfile_recorder.cpp)
add_dependencies(bag_file_node ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bag_file_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(bag_file_node PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(PROGRAMS scripts/generate_urdf.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
        )        
catkin_install_python(PROGRAMS
        src/command_publishers/cmdvel_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ) 
catkin_install_python(PROGRAMS
        src/command_publishers/mode_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )   
catkin_install_python(PROGRAMS
        src/command_publishers/goal_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )             