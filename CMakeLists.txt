cmake_minimum_required(VERSION 2.8)
project(These)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#Tests to compile
set (TESTS_DIRECTORIES
    SmartPointer
    Symbolic
    Any
    LWPLS
    Random
    Plot
)

#Build tests
foreach(f ${TESTS_DIRECTORIES})
    add_executable(
        tests${f}
        ${f}/tests/mainTests.cpp
    )
endforeach(f) 

