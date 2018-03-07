SET(PROJECT_NAME SFM)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

set(BUILD_TYPE Release)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# required libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

#find_package(Boost COMPONENTS system thread)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS_RELEASE} ${SSE_FLAGS} -std=c++0x"
)


include_directories(
  ${PROJECT_SOURCE_DIR}/inc
  ${PROJECT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


set(SFM_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/SFM.cpp
  ${PROJECT_SOURCE_DIR}/src/Math.cpp
  ${PROJECT_SOURCE_DIR}/src/Track.cpp
  ${PROJECT_SOURCE_DIR}/src/Camera.cpp
)
add_library(SFM_libs ${SFM_SOURCE_FILES})

add_executable(SFM ${PROJECT_SOURCE_DIR}/test.cpp)

target_link_libraries(SFM
                      SFM_libs
                    ${OpenCV_LIBS}
                    ${CERES_LIBRARIES})
