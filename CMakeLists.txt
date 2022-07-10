cmake_minimum_required(VERSION 3.5)
project(stereo_image_saver)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        message_filters
        roscpp
        sensor_msgs
        std_msgs
        )

catkin_package()

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS filesystem)
include_directories(${Boost_filesystem_DIR})

add_executable(stereo_image_saver_node src/stereo_image_saver.cpp)
target_link_libraries(stereo_image_saver_node ${catkin_LIBRARIES} ${OpenCV_LIBS} Boost::filesystem)
