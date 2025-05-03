cmake_minimum_required(VERSION 3.15)
project(Frogs)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Enable optimization flags for Release mode
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -march=native -funroll-loops)
endif()


set(DEPENDENCIES
        include/case.hpp
        src/case.cpp
        include/command_line.hpp
        src/command_line.cpp
        include/parameters.hpp
        include/preprocessor.hpp
        src/preprocessor.cpp
        external/include/CircleSector.h
        include/individual.hpp
        src/individual.cpp
        include/follower.hpp
        src/follower.cpp
        include/stats_interface.hpp
        src/stats_interface.cpp
        include/heuristic_interface.hpp
        include/lahc.hpp
        src/lahc.cpp
        external/include/magic_enum.hpp
        include/leader_array.hpp
        src/leader_array.cpp
        include/solution.hpp
        src/solution.cpp
        include/initializer.hpp
        src/initializer.cpp
        include/cbma.hpp
        src/cbma.cpp
        include/sga.hpp
        src/sga.cpp
        include/leader_sga.hpp
        src/leader_sga.cpp
)

add_executable(Run main.cpp ${DEPENDENCIES})

# Include directories
include_directories(include external/include)

# Find and link OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_link_libraries(Run PRIVATE OpenMP::OpenMP_CXX)
endif()

# Find and link MPI (only in Release)
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    find_package(MPI REQUIRED)
    if (MPI_CXX_FOUND)
        target_include_directories(Run PRIVATE ${MPI_INCLUDE_PATH})
        target_link_libraries(Run PRIVATE MPI::MPI_CXX)
    endif()
endif()

# Testing setup for Debug mode only
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_subdirectory(external/googletest)
    enable_testing()
    add_executable(Tests tests/test_main.cpp
            ${DEPENDENCIES}
            tests/case_test.cpp
            tests/command_line_test.cpp
            tests/preprocessor_test.cpp
            tests/individual_test.cpp
            tests/follower_test.cpp
            tests/lahc_test.cpp
            tests/leader_array_test.cpp
            tests/cbma_test.cpp
            tests/sga_test.cpp
            tests/leader_sga_test.cpp)

    target_include_directories(Tests PRIVATE include external/include)
    target_link_libraries(Tests PRIVATE gtest gtest_main OpenMP::OpenMP_CXX)

    add_custom_target(valgrind_tests
            COMMAND valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file=valgrind_tests.log ./Tests
            DEPENDS Tests
            COMMENT "Running Valgrind on test executable...")

    add_test(NAME FrogsTests COMMAND Tests)
endif()
