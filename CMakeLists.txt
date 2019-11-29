cmake_minimum_required(VERSION 2.8)
project(PosEstimate)

#IF(NOT CMAKE_BUILD_TYPE)
SET(CMAKE_BUILD_TYPE DEBUG)
#ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w -g -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -g -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Cholmod REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/PosEstimate/include
${PROJECT_SOURCE_DIR}/PosEstimate
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${CHOLMOD_INCLUDE_DIR}
# ${PROJECT_SOURCE_DIR}/../ThirdParty/
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
PosEstimate/sources/CFuncHelper.cpp
PosEstimate/sources/CMap.cpp
PosEstimate/sources/COptimizer.cpp
PosEstimate/sources/CFrame.cpp
PosEstimate/sources/CFeatureTrack.cpp
PosEstimate/sources/CORBextractor.cpp
PosEstimate/sources/CORBmatcher.cpp
PosEstimate/sources/CMapDrawer.cpp
PosEstimate/sources/CFrameDrawer.cpp
PosEstimate/sources/CViewer.cpp
PosEstimate/sources/CDataManager.cpp
PosEstimate/IMU/IMUPreintegrator.cpp
PosEstimate/IMU/imudata.cpp
PosEstimate/IMU/so3.cpp
# PosEstimate/IMU/g2otypes.cpp
PosEstimate/IMU/NavState.cpp
source/M_DataLoader.cpp
source/M_Config.cpp
source/M_CoorTransform.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
cholmod
${CHOLMOD_LIBRARIES}
#${PROJECT_SOURCE_DIR}/ThirdParty/DBoW2/lib/libDBoW2.so
# ${PROJECT_SOURCE_DIR}/../ThirdParty/g2o/lib/libg2o.so
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Output/)

# add_executable(WeiyaDataPreProcess
# PosEstimate/main.cpp)
# target_link_libraries(WeiyaDataPreProcess ${PROJECT_NAME})

add_executable(PosEstEx
PosEstimate/main.cpp)
target_link_libraries(PosEstEx ${PROJECT_NAME})