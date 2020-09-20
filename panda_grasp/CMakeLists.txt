cmake_minimum_required(VERSION 2.8.3)
project(panda_grasp)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages 
find_package(
  catkin REQUIRED
  COMPONENTS cmake_modules
             cv_bridge
             eigen_conversions
             geometry_msgs
             message_generation
             pcl_conversions
             roscpp
             sensor_msgs
             std_msgs
             visualization_msgs
             tf
             agile_grasp
             tf2_ros
             tf2_eigen
             tf2_geometry_msgs) # learn_service_pkg

# System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)
find_package(LAPACK REQUIRED)
find_package(OpenCV 3.4 REQUIRED)
find_package(PCL 1.9.1 EXACT REQUIRED)

set(INC_DIR /usr/local/include)
set(LINK_DIR /usr/local/lib)
# find_package(gpd)

# set(OpenCV "/usr/local/gpd") ~ find_package(VTK 6.0 REQUIRED NO_MODULE) #
# required in Fedora Core

# Set compiler optimization flags set(CMAKE_CXX_FLAGS "-DNDEBUG -O3 -fopenmp
# -Wno-deprecated -Wenum-compare")

# Uncomment this if the package has a setup.py. This macro ensures modules and
# global scripts declared therein get installed See
# http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ##############################################################################
# Declare ROS messages, services and actions ##
# ##############################################################################

# To declare and build messages, services or actions from within this package,
# follow these steps: * Let MSG_DEP_SET be the set of packages whose message
# types you use in your messages/services/actions (e.g. std_msgs,
# actionlib_msgs, ...). * In the file package.xml: * add a build_depend and a
# run_depend tag for each package in MSG_DEP_SET * If MSG_DEP_SET isn't empty
# the following dependencies might have been pulled in transitively but can be
# declared for certainty nonetheless: * add a build_depend tag for
# "message_generation" * add a run_depend tag for "message_runtime" * In this
# file (CMakeLists.txt): * add "message_generation" and every package in
# MSG_DEP_SET to find_package(catkin REQUIRED COMPONENTS ...) * add
# "message_runtime" and every package in MSG_DEP_SET to
# catkin_package(CATKIN_DEPENDS ...) * uncomment the add_*_files sections below
# as needed and list every .msg/.srv/.action file to be processed * uncomment
# the generate_messages entry below * add every package in MSG_DEP_SET to
# generate_messages(DEPENDENCIES ...)

# Generate services in the 'srv' folder
add_service_files(FILES MyGraspPose.srv)
# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
# The catkin_package macro generates cmake config files for your package Declare
# things to be passed to dependent projects INCLUDE_DIRS: uncomment this if you
# package contains header files LIBRARIES: libraries you create in this project
# that dependent projects also need CATKIN_DEPENDS: catkin_packages dependent
# projects also need DEPENDS: system dependencies of this project that dependent
# projects also need
catkin_package(
  INCLUDE_DIRS
  include
  # LIBRARIES
  CATKIN_DEPENDS
    # cmake_modules
    # cv_bridge
    # eigen_conversions
    # geometry_msgs
    message_runtime
    # pcl_conversions
    # roscpp
    # sensor_msgs
    # std_msgs
    # visualization_msgs
    # tf2_geometry_msgs
  # DEPENDS
  #   Eigen
  #   LAPACK
  #   OpenCV
  #   PCL
  )


# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS} ${INC_DIR})
link_directories(${LINK_DIR})
# ~ include(${VTK_USE_FILE}) # required in Fedora Core

add_executable(transform_point_cloud src/transform_point_cloud.cpp)
add_dependencies(transform_point_cloud MyGraspPose_generate_messages_cpp)
target_link_libraries(transform_point_cloud ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(agile_grasp_server src/agile_grasp_server.cpp)
add_dependencies(agile_grasp_server MyGraspPose_generate_messages_cpp)
target_link_libraries(agile_grasp_server config_file ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(gpd_grasp_server src/gpd_grasp_server.cpp)
add_dependencies(gpd_grasp_server MyGraspPose_generate_messages_cpp)
target_link_libraries(gpd_grasp_server gpd ${catkin_LIBRARIES} ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES})

# ##############################################################################
# Install ##
# ##############################################################################

# Mark cpp header files for installation install(DIRECTORY
# include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h" PATTERN ".git" EXCLUDE)

# ## install launch files install(DIRECTORY launch/ DESTINATION
# ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
