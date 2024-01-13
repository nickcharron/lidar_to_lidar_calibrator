cmake_minimum_required(VERSION 3.2)
project(lidar_to_lidar_calibrator)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.11.1 REQUIRED)
find_package(beam REQUIRED calibration mapping utils optimization filtering matching)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(OpenCV 4.5.2 REQUIRED COMPONENTS)

# Add the core inspection library
add_subdirectory(src/lib)

# extract images binary
add_executable(${PROJECT_NAME}_main
	src/CalibrateLidarToLidar.cpp
)
target_link_libraries(${PROJECT_NAME}_main
	${PROJECT_NAME}
	gflags
	beam::utils)


################ tests ##################
add_executable(${PROJECT_NAME}_tests
tests/Tests.cpp
)

target_include_directories(${PROJECT_NAME}_tests
	PUBLIC
  	include
)

target_link_libraries(${PROJECT_NAME}_tests
	${PROJECT_NAME}
	Catch2::Catch2
	beam::utils
	beam::optimization
)
