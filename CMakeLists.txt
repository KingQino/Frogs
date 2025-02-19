cmake_minimum_required(VERSION 3.15)
project(Frogs)

set(CMAKE_CXX_STANDARD 17)

# Enable optimization flags for Release mode
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
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
        external/include/Split.h
        external/src/Split.cpp
        external/include/LocalSearch.h
        external/src/LocalSearch.cpp
)

add_executable(Run main.cpp ${DEPENDENCIES})

# Include directories
target_include_directories(Run PRIVATE include)

# Enable multithreading if needed
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(Run PRIVATE Threads::Threads)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_subdirectory(external/googletest) # Use pre-existing GoogleTest
    include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

    enable_testing()
    add_executable(Tests tests/test_main.cpp
            ${DEPENDENCIES}
            tests/case_test.cpp
            tests/command_line_test.cpp
            tests/preprocessor_test.cpp
            tests/individual_test.cpp)

    target_include_directories(Tests PRIVATE include external/include)
    target_link_libraries(Tests PRIVATE gtest gtest_main)

    # Register tests
    add_test(NAME FrogsTests COMMAND Tests)
endif()

