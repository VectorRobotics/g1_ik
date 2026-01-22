cmake_minimum_required(VERSION 3.8)
project(g1_ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(CMakeDependentOption)

add_compile_options(-w -Wall -Wextra -Wpedantic)

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install" CACHE PATH "Install path prefix" FORCE)
set(CMAKE_INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib)
include_directories(BEFORE SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/third_party/include)

# ---- Options ---- #
set(USE_CASADI ON CACHE BOOL "Use CasADi for optimization")
option(BUILD_TESTING "Build tests" ON)
cmake_dependent_option(
    INSTALL_TESTING "Install test sources alongside the project" ON
    "BUILD_TESTING" OFF
)

# ---- Dependencies ---- #

list(INSERT CMAKE_PREFIX_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

find_package(pinocchio REQUIRED)

if(USE_CASADI)
    find_package(casadi REQUIRED)
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen/eigen")

# ---- Library ---- #
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_library(g1_ik ${SOURCES})

target_include_directories(g1_ik PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(g1_ik
    PUBLIC
      Eigen3::Eigen
      pinocchio::pinocchio
)

if(USE_CASADI)
    target_link_libraries(g1_ik
    PUBLIC
        casadi
    )
    target_compile_definitions(g1_ik PRIVATE USE_CASADI)
endif()

# ---- Tests ---- #

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    
    add_executable(test_ik test/test_ik.cpp)
    add_executable(test_robot_arm_ik test/test_robot_arm_ik.cpp)
    target_link_libraries(test_ik PRIVATE g1_ik GTest::gtest_main)
    target_link_libraries(test_robot_arm_ik PRIVATE g1_ik GTest::gtest_main)

    if(USE_CASADI)
        target_compile_definitions(test_robot_arm_ik PRIVATE USE_CASADI)
        target_compile_definitions(test_ik PRIVATE USE_CASADI)
    endif()

    # Allows running 'ctest' from the build directory
    include(GoogleTest)
    gtest_discover_tests(test_robot_arm_ik)
    gtest_discover_tests(test_ik)

endif()

# --- Installation --- #

include(GNUInstallDirs)

# Install the library
install(TARGETS g1_ik
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install the headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Install the test source (as requested)
if(INSTALL_TESTING)
    install(FILES test/test_ik.cpp DESTINATION share/${PROJECT_NAME}/test)
    install(FILES test/test_robot_arm_ik.cpp DESTINATION share/${PROJECT_NAME}/test)
endif()