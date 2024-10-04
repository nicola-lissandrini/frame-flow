cmake_minimum_required(VERSION 3.5)

project(tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package (nlib2 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(../src ${EIGEN3_INCLUDE_DIRS} ${nlib2_INCLUDE_DIRS})

add_executable(tests tests.cpp ../src/frameflow.cpp)
target_link_libraries (tests gtest_main)


enable_testing ()

add_test(NAME algorithm_result_test COMMAND AlgorithmResultTest)
