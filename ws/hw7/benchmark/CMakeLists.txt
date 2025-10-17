cmake_minimum_required(VERSION 3.8)

add_executable(prm_benchmark-hw7
    prm_benchmark.cpp
    ../MySamplingBasedPlanners.cpp
    # Add other required sources if needed
)

# Link required libraries
# Replace 'AMPCorePrivate' and 'AMPCorePublic' with your actual core library targets if different
# You may need to adjust these for your project

find_package(Eigen3 3.3.4 REQUIRED NO_MODULE)

# If you have yaml-cpp, AMPCorePrivate, AMPCorePublic targets, link them
# If not, remove or adjust these lines

# These variables should be set in your parent CMakeLists.txt
# AMP_INCLUDE_DIRS, AMP_DEPENDENCY_INCLUDE_DIRS

# Link libraries
# You may need to add/remove libraries depending on your project

# Example:
target_link_libraries(prm_benchmark-hw7 PRIVATE
    yaml-cpp
    Eigen3::Eigen
    AMPCorePrivate
    AMPCorePublic
)

target_include_directories(prm_benchmark-hw7 PUBLIC
    ${AMP_INCLUDE_DIRS}
    ${AMP_DEPENDENCY_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)
