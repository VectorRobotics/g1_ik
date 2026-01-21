cmake_minimum_required(VERSION 3.8)
project(g1_ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-w -Wall -Wextra -Wpedantic)

# Set a default install prefix if the user didn't specify one
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install" CACHE PATH "Install path prefix" FORCE)

list(INSERT CMAKE_PREFIX_PATH 0 "/opt/openrobots")

set(pinocchio_DIR "/opt/openrobots/lib/cmake/pinocchio")

find_package(pinocchio REQUIRED)

set(USE_CASADI ON CACHE BOOL "Use CasADi for optimization")
if(USE_CASADI)
    find_package(casadi REQUIRED)
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen/eigen")

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_library(g1_ik ${SOURCES})

target_include_directories(g1_ik PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    if(USE_CASADI)
        ${hpp-fcl_INCLUDE_DIRS}
        ${coal_INCLUDE_DIRS}
        ${casadi_INCLUDE_DIR}
    endif()
    ${pinocchio_INCLUDE_DIR}
)

target_link_directories(g1_ik PUBLIC 
    if(USE_CASADI)
        ${hpp-fcl_LIBRARY_DIRS}
        ${coal_LIBRARY_DIRS}
        ${casadi_LIB_DIR} 
    endif()
    ${pinocchio_LIB_DIR}
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
endif()

if(USE_CASADI)
    target_compile_definitions(g1_ik PRIVATE USE_CASADI)
endif()

option(BUILD_TESTING "Build tests" ON)
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

# --- Installation ---
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
install(FILES test/test_ik.cpp DESTINATION share/${PROJECT_NAME}/test)