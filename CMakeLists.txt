cmake_minimum_required(VERSION 3.8)
project(g1_ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-w -Wall -Wextra -Wpedantic)

set(pinocchio_DIR "/opt/openrobots/lib/cmake/pinocchio")
# set(pinocchio_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pinocchio/install/lib")
# set(pinocchio_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pinocchio/install/include")
# set(PINOCCHIO_URDF_LIB "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pinocchio/install/lib/libpinocchio_parsers.so")
# set(casadi_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/casadi/cmake")
# set(casadi_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/casadi/")
# set(casadi_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/casadi/include")
# set(hpp-fcl_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/coal/install/lib/cmake/hpp-fcl")
# set(coal_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/coal/install/lib/cmake/coal")

find_package(pinocchio REQUIRED)

set(USE_CASADI OFF CACHE BOOL "Use CasADi for optimization")

if(USE_CASADI)
    set(casadi_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/casadi/cmake")
    set(casadi_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/casadi/")
    set(casadi_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/casadi/include")
    set(hpp-fcl_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/coal/install/lib/cmake/hpp-fcl")
    set(coal_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/coal/install/lib/cmake/coal")
    set(hpp-fcl_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/coal/install/lib/cmake/hpp-fcl")
    
    find_package(casadi REQUIRED)
endif()
find_library(PINOCCHIO_URDF_LIB 
    NAMES pinocchio_urdf 
    PATHS "${pinocchio_LIB_DIR}" 
    NO_DEFAULT_PATH
)

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

option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    
    add_executable(test_ik test/test_ik.cpp)
    add_executable(test_robot_arm_ik test/test_robot_arm_ik.cpp)
    target_link_libraries(test_ik PRIVATE g1_ik GTest::gtest_main)
    target_link_libraries(test_robot_arm_ik PRIVATE g1_ik GTest::gtest_main)

    # Allows running 'ctest' from the build directory
    include(GoogleTest)
    gtest_discover_tests(test_robot_arm_ik)
    # gtest_discover_tests(test_ik)

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