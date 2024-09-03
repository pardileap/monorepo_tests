cmake_minimum_required(VERSION 3.10) 

#Set the C++ standard
set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED True)


enable_testing()

# Add test files
file(GLOB_RECURSE TEST_FILES "*.cpp")

# Add test executable
add_executable(run_tests ${TEST_FILES})

# Link Google Test libraries
target_link_libraries(run_tests gtest gtest_main)

# Define tests
add_test(NAME MyTests COMMAND run_tests)