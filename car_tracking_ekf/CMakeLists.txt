cmake_minimum_required(VERSION 2.8.3)
project(car_tracking_ekf)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_executable(car_tracking
    src/main.cpp src/car_tracking_ekf.cpp 
)

add_dependencies(car_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(car_tracking
    ${catkin_LIBRARIES}
)
