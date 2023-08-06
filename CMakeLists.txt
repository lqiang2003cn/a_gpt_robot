cmake_minimum_required(VERSION 2.8.3)
project(a_gpt_robot)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    aruco
)

catkin_package(
    CATKIN_DEPENDS aruco
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(make_marker src/make_marker.cpp)
target_link_libraries(make_marker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(cv_aruco_gen src/cv_aruco_gen.cpp)
target_link_libraries(cv_aruco_gen ${OpenCV_LIBRARIES})