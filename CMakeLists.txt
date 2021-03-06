cmake_minimum_required(VERSION 2.8)
project(These)

find_package(PkgConfig REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_package (Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(SFML REQUIRED system window graphics)
include_directories(${SFML_INCLUDE_DIR})


pkg_check_modules(JSONCPP jsoncpp)
include_directories(${JSONCPP_INCLUDE_DIRS})
link_libraries(${JSONCPP_LIBRARIES})

message("JSONCPP FOUND CONFIG: ${JSONCPP_CONFIG}")
message("JSONCPP INCLUDE DIRS: ${JSONCPP_INCLUDE_DIRS}")
message("JSONCPP LIBRARIES: ${JSONCPP_LIBRARIES}")


pkg_check_modules(CMAES libcmaes)
message("CMAES FOUND CONFIG: ${CMAES_CONFIG}")
message("CMAES INCLUDE DIRS: ${CMAES_INCLUDE_DIRS}")
message("CMAES LIBRARIES: ${CMAES_LIBRARIES}")
include_directories(${CMAES_INCLUDE_DIRS})
link_libraries(${CMAES_LIBRARIES})



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
    Model
)




#Enable C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#Special flags
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
#optim
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -O2 -funroll-loops -march=native -mfpmath=sse")



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
	testCamSimple
        Simulations/camTestSimple.cpp
)



add_executable(
	testCompass
        Simulations/compassTest.cpp
)

add_executable(
	testMonopole
        Simulations/monopoleTest.cpp
)


add_executable(
	SimpleWalkerOptim
        Simulations/SimpleWalkerOptim.cpp
)

add_executable(
	RoundFootOptim
        Simulations/RoundFootOptim.cpp
)


add_executable(
	testSpring
        Simulations/springTest.cpp
)


add_executable(
	testRoundFoot
        Simulations/roundFootTest.cpp
)


add_executable(
	testRoundFeetCompass
        Simulations/roundFeetCompassTest.cpp
)

add_executable(
	KneeWalkerOptim
        Simulations/KneeWalkerOptim.cpp
)
add_executable(
	KneeWalkerOptimAll
        Simulations/KneeWalkerOptimAll.cpp
)
add_executable(
	KneeWalkerOptimEigen
        Simulations/KneeWalkerOptimEigen.cpp
)
add_executable(
	testStability
        Simulations/testStability.cpp
)
add_executable(
	testCamSpring
        Simulations/camTestSpring.cpp
)

add_executable(
	testCamWalker
        Model/testCamWalker.cpp
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

target_link_libraries(testCamSimple
    sfml-graphics
    sfml-window
    sfml-system
)

target_link_libraries(testCompass
    sfml-graphics
    sfml-window
    sfml-system
)

target_link_libraries(testMonopole
    sfml-graphics
    sfml-window
    sfml-system
)


target_link_libraries(testSpring
    sfml-graphics
    sfml-window
    sfml-system
)


target_link_libraries(testRoundFoot
    sfml-graphics
    sfml-window
    sfml-system
)


target_link_libraries(SimpleWalkerOptim
    sfml-graphics
    sfml-window
    sfml-system
    cmaes
)

target_link_libraries(testRoundFeetCompass
    sfml-graphics
    sfml-window
    sfml-system
)

target_link_libraries(RoundFootOptim
    sfml-graphics
    sfml-window
    sfml-system
    cmaes
)

target_link_libraries(testsModel
    sfml-graphics
    sfml-window
    sfml-system
    jsoncpp
)

target_link_libraries(KneeWalkerOptim
    sfml-graphics
    sfml-window
    sfml-system
    jsoncpp
    cmaes
)


target_link_libraries(KneeWalkerOptimAll
    sfml-graphics
    sfml-window
    sfml-system
    jsoncpp
    cmaes
)

target_link_libraries(KneeWalkerOptimEigen
    sfml-graphics
    sfml-window
    sfml-system
    jsoncpp
    cmaes
)

target_link_libraries(testStability
    sfml-graphics
    sfml-window
    sfml-system
    jsoncpp
)


target_link_libraries(testCamSpring
    sfml-graphics
    sfml-window
    sfml-system
)


target_link_libraries(testCamWalker
    sfml-graphics
    sfml-window
    sfml-system
    jsoncpp
)
