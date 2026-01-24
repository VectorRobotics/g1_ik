cmake_minimum_required(VERSION 3.8)
project(g1_ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(CMakeDependentOption)

add_compile_options(-w -Wall -Wextra -Wpedantic)

# ---- Integration & Path Logic ---- #

if(DEFINED ENV{COLCON_PREFIX_PATH})
    set(ROS2_DEFAULT ON)
else()
    set(ROS2_DEFAULT OFF)
endif()

option(ROS2_INTEGRATION "Enable ROS 2 integration" ${ROS2_DEFAULT})

if(NOT ROS2_INTEGRATION)
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install" CACHE PATH "Local install prefix" FORCE)
    endif()
endif()

set(CMAKE_INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib)
include_directories(BEFORE SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/third_party/include)

# ---- Options & Dependencies ---- #

set(USE_CASADI OFF CACHE BOOL "Use CasADi for optimization")
option(BUILD_TESTING "Build tests" ON)
cmake_dependent_option(INSTALL_TESTING "Install test sources" ON "BUILD_TESTING" OFF)

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

target_link_libraries(g1_ik PUBLIC Eigen3::Eigen pinocchio::pinocchio)

if(USE_CASADI)
    target_link_libraries(g1_ik PUBLIC casadi)
    target_compile_definitions(g1_ik PRIVATE USE_CASADI)
endif()

# ---- Testing ---- #

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    
    add_executable(test_ik test/test_ik.cpp)
    add_executable(test_robot_arm_ik test/test_robot_arm_ik.cpp)
    target_link_libraries(test_ik PRIVATE g1_ik GTest::gtest_main)
    target_link_libraries(test_robot_arm_ik PRIVATE g1_ik GTest::gtest_main)

    if(USE_CASADI)
        target_compile_definitions(test_ik PRIVATE USE_CASADI)
        target_compile_definitions(test_robot_arm_ik PRIVATE USE_CASADI)
    endif()

    include(GoogleTest)
    gtest_discover_tests(test_ik)
    gtest_discover_tests(test_robot_arm_ik)
endif()

# --- Installation --- #

include(GNUInstallDirs)

install(TARGETS g1_ik
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(INSTALL_TESTING)
    install(FILES test/test_ik.cpp test/test_robot_arm_ik.cpp DESTINATION share/${PROJECT_NAME}/test)
endif()

# ---- ROS 2 Integration ---- #

if(ROS2_INTEGRATION)
    find_package(ament_cmake REQUIRED)

    install(DIRECTORY launch assets DESTINATION share/${PROJECT_NAME})

    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()

    ament_package()
endif()