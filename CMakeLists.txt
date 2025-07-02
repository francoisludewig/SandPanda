cmake_minimum_required(VERSION 3.1.2)

project(SandPanda VERSION 1.1.0)

set(SOURCES_FILES
        Sources/Contact/ContactDetection.cpp
        Sources/Contact/Contact.cpp
        Sources/Contact/ContactDetectorSphereVersusCone.cpp
        Sources/Contact/ContactDetectorSphereVersusDisk.cpp
        Sources/Contact/ContactDetectorBodyVersusBody.cpp
        Sources/Contact/ContactDetectorSphereVersusElbow.cpp
        Sources/Contact/ContactDetectorBodyVersusCone.cpp
        Sources/Contact/ContactDetectorSphereVersusPlan.cpp
        Sources/Contact/ContactDetectorBodyVersusDisk.cpp
        Sources/Contact/ContactDetectorSphereVersusSphere.cpp
        Sources/Contact/ContactDetectorBodyVersusPlan.cpp
        Sources/Contact/ContactDetectorBodyVersusElbow.cpp
        Sources/Contact/ContactDetectorSphereVersusBody.cpp


        Sources/Elongations/Elongation.cpp


        Sources/Solids/HollowBall.cpp
        Sources/Solids/Body.cpp
        Sources/Solids/MasterSolid.cpp
        Sources/Solids/BodySpecie.cpp
        Sources/Solids/Plan.cpp
        Sources/Solids/Cone.cpp
        Sources/Solids/PlanR.cpp
        Sources/Solids/Elbow.cpp
        Sources/Solids/Solid.cpp
        Sources/Solids/Sphere.cpp
        Sources/Solids/Velocity.cpp
        Sources/Solids/Sinusoid.cpp
        Sources/LinkedCells/LinkedCellFiller.cpp
        Sources/LinkedCells/SolidCellsBuilder.cpp

        Sources/Dynamic/Move.cpp
        Sources/Configuration/Option.cpp
        Sources/Dynamic/Compaction.cpp
        Sources/Dynamic/Periodicity.cpp
        Sources/ComputingForce.cpp
        Sources/Dynamic/PowderPaQ.cpp
        Sources/Repository/ReadWrite.cpp
        Sources/Configuration/Configuration.cpp
        Sources/Configuration/Monitoring.cpp
        Sources/Dynamic/Evolution.cpp
        Sources/Configuration/Gravity.cpp
        Sources/Repository/SimulationDataManager.cpp
        Sources/main.cpp
        Sources/Elongations/ElongationManager.cpp
        Sources/Contact/ContactIdentifier.cpp
)

set(INCLUDES_FILES
        Includes/Contact/ContactDetection.h
        Includes/Contact/Contact.h
        Includes/Contact/ContactDetectorSphereVersusCone.h
        Includes/Contact/ContactDetectorSphereVersusDisk.h
        Includes/Contact/ContactDetectorBodyVersusBody.h
        Includes/Contact/ContactDetectorSphereVersusElbow.h
        Includes/Contact/ContactDetectorBodyVersusCone.h
        Includes/Contact/ContactDetectorSphereVersusPlan.h
        Includes/Contact/ContactDetectorBodyVersusDisk.h
        Includes/Contact/ContactDetectorSphereVersusSphere.h
        Includes/Contact/ContactDetectorBodyVersusPlan.h
        Includes/Contact/ContactDetectorBodyVersusElbow.h
        Includes/Contact/ContactDetectorSphereVersusBody.h

        Includes/Elongations/Elongation.h
        Includes/Elongations/ElongationManager.h

        Includes/Solids/Body.h
        Includes/Solids/MasterSolid.h
        Includes/Solids/BodySpecie.h
        Includes/Solids/Cone.h
        Includes/Solids/Plan.h
        Includes/Solids/PlanR.h
        Includes/Solids/Elbow.h
        Includes/Solids/Solid.h
        Includes/Solids/Sphere.h
        Includes/Solids/Velocity.h
        Includes/Solids/Sinusoid.h
        Includes/Solids/MechanicalPoint.h
        Includes/Solids/HollowBall.h
        Includes/Repository/SimulationData.h

        Includes/LinkedCells/LinkedCellFiller.h
        Includes/LinkedCells/SolidCellsBuilder.h

        Includes/Dynamic/Move.h
        Includes/Dynamic/Compaction.h
        Includes/Configuration/Option.h
        Includes/Configuration/Monitoring.h
        Includes/ComputingForce.h
        Includes/Dynamic/Periodicity.h
        Includes/Dynamic/PowderPaQ.h
        Includes/Configuration/Configuration.h
        Includes/Repository/ReadWrite.h
        Includes/Dynamic/Evolution.h
        Includes/Configuration/Gravity.h
        Includes/Repository/SimulationDataManager.h
        Includes/Contact/ContactIdentifier.h
)

set(SandPanda_FILES
        ${INCLUDES_FILES}
        ${SOURCES_FILES}
)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenMP REQUIRED)

add_executable(
        SandPanda
        ${SandPanda_FILES}
)

target_link_libraries(
        SandPanda
        ${GTEST_LIBRARIES}
        ${OpenMP_}
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "-O0 -g -pg -fopenmp -fno-omit-frame-pointer")
else ()
    set(CMAKE_CXX_FLAGS "-O3 -std=c++11 -fopenmp")
endif ()

if (OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()


enable_testing()

#add_subdirectory("Tests")
