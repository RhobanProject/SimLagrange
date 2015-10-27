cmake_minimum_required(VERSION 2.8)
project(These)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#Tests to compile
set(TESTS_DIRECTORIES
    SmartPointer
    Symbolic
    Any
    Random
    Plot
    Vector
    SimViewer
    VectorMap
    SymOptim
    SimMecha
)


#Include Eigen (linear algebra template library)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Eigen)

#Include SFML (windows and graphic library)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/SFML/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/SFML/lib)

#Enable C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


#Build tests
foreach(f ${TESTS_DIRECTORIES})
    add_executable(
        tests${f}
        ${f}/tests/mainTests.cpp
    )
endforeach(f)


add_executable(
	testCam
        Simulations/camTest.cpp
)


add_executable(
	testCompass
        Simulations/compassTest.cpp
)


add_executable(
	testSpring
        Simulations/springTest.cpp
)


#Link libraries
target_link_libraries(testsSimViewer
    sfml-graphics
    sfml-window
    sfml-system
)
target_link_libraries(testsSimMecha
    sfml-graphics
    sfml-window
    sfml-system
)

target_link_libraries(testCam
    sfml-graphics
    sfml-window
    sfml-system
)

target_link_libraries(testCompass
    sfml-graphics
    sfml-window
    sfml-system
)


target_link_libraries(testSpring
    sfml-graphics
    sfml-window
    sfml-system
)
