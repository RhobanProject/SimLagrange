cmake_minimum_required(VERSION 2.8)
project(These)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#Tests to compile
set (TESTS_DIRECTORIES
    SmartPointer
    Symbolic
    Any
    Random
    Plot
    Vector
    SimViewer
)

#Include Eigen (linear algebra template library)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Eigen)

#Include SFML (windows and graphic library)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/SFML/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/SFML/lib)

#Build tests
foreach(f ${TESTS_DIRECTORIES})
    add_executable(
        tests${f}
        ${f}/tests/mainTests.cpp
    )
endforeach(f) 

#Link libraries
target_link_libraries(testsSimViewer 
    sfml-graphics 
    sfml-window 
    sfml-system
)

