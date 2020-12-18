cmake_minimum_required(VERSION 2.8)
project(UWB_IMU_WheelEncoder_Fuse)

SET(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -O0")

find_package(OpenCV 3.0 REQUIRED)
find_package(Eigen3 REQUIRED)

set(G2O_LIBS ${PROJECT_SOURCE_DIR}/ThirdParty/g2o/build/libg2o_line_merge.a)
message(”G2O_LIBS Path “ ${G2O_LIBS})

include_directories(
     ${PROJECT_SOURCE_DIR}
     ${PROJECT_SOURCE_DIR}/include
     ${PROJECT_INCLUDE_DIRS}
     ${PROJECT_SOURCE_DIR}/ThirdParty/g2o
     ${PROJECT_SOURCE_DIR}/ThirdParty/eigen3

)
message(${OpenCV_INCLUDE_DIRS})

add_executable(UWB_FUSE main.cpp
             src/SerialPort.cpp
             src/readData.cpp
             src/utils.cpp
             src/Condensation.cpp
             src/UKF.cpp
             src/CoreAlgorithm.cpp
             src/calibrate.cpp
)

target_link_libraries(UWB_FUSE
        ${OpenCV_LIBS}
        ${G2O_LIBS}
        )
