cmake_minimum_required(VERSION 3.5)
project(dobby_node)

set(MRobotics_DIR "$ENV{HOME}/ModernRoboticsCpp")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MRobotics_DIR}/include
)


# find dependencies
find_package (Eigen3 REQUIRED)
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

if (APPLE)
    # default library path
    message(STATUS "Checking Eigen availability...")
    if (NOT EXISTS ${EIGEN3_INCLUDE_DIR})
        message(WARNING "Unable to find EIGEN3_INCLUDE_DIR...")
        message(STATUS "Installing Eigen...")
        # now we need to download the Eigen library
        execute_process(COMMAND brew install eigen)
    endif()
endif()


if (WIN32)
  SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
  INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
  message(STATUS "${EIGEN3_INCLUDE_DIR}")
endif()

# Finding Eigen is somewhat complicated because of our need to support Ubuntu
# all the way back to saucy.  First we look for the Eigen3 cmake module
# provided by the libeigen3-dev on newer Ubuntu.  If that fails, then we
# fall-back to the version provided by cmake_modules, which is a stand-in.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

# Note that eigen 3.2 (on Ubuntu Wily) only provides EIGEN3_INCLUDE_DIR,
# not EIGEN3_INCLUDE_DIRS, so we have to set the latter from the former.
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(include
                    ${EIGEN3_INCLUDE_DIRS})
                    

                    

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(odom_pub src/odom_pub.cpp)
ament_target_dependencies(odom_pub rclcpp geometry_msgs std_msgs nav_msgs)

add_executable(odom_to_tf src/odom_to_tf.cpp)
ament_target_dependencies(odom_to_tf rclcpp tf2_ros nav_msgs geometry_msgs)

install(TARGETS
odom_pub
odom_to_tf
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
