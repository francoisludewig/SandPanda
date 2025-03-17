cmake_minimum_required(VERSION 3.6)

project(SandPanda VERSION 1.0.0)

set(SOURCES_FILES
        Sources/Object/HollowBall.cpp
        Sources/Object/Body.cpp
        Sources/Object/MasterSolid.cpp
        Sources/Object/BodySpecie.cpp
        Sources/Object/Velocity.cpp
        Sources/Object/Sinusoid.cpp
        Sources/Object/Plan.cpp
        Sources/Object/Cone.cpp
        Sources/Object/PlanR.cpp
        Sources/Object/Elbow.cpp
        Sources/Object/Solid.cpp
        Sources/Object/Sphere.cpp
		Sources/Object/Periodicity.cpp

		Sources/Configuration/Option.cpp
		Sources/Configuration/Data.cpp
		Sources/Configuration/Gravity.cpp
		
		Sources/Contact/Contact.cpp		
		Sources/Contact/ContactDetection.cpp	
		Sources/Contact/Elongation.cpp
		Sources/Contact/ComputingForce.cpp

		Sources/Dynamic/Move.cpp
		Sources/Dynamic/Compaction.cpp
		Sources/Dynamic/PowderPaQ.cpp
		Sources/Dynamic/Evolution.cpp
		
		Sources/Repository/ReadWrite.cpp
		
		Sources/main.cpp
)

set(INCLUDES_FILES
        Includes/Object/HollowBall.h
        Includes/Object/Body.h
        Includes/Object/MasterSolid.h
        Includes/Object/BodySpecie.h
        Includes/Object/Cone.h
        Includes/Object/Plan.h
        Includes/Object/PlanR.h
        Includes/Object/Elbow.h
        Includes/Object/Solid.h
        Includes/Object/Sphere.h
        Includes/Object/Velocity.h
        Includes/Object/Sinusoid.h		
		Includes/Object/Periodicity.h
		
		Includes/Configuration/Gravity.h
		Includes/Configuration/Option.h
		Includes/Configuration/Data.h	
		
		Includes/Contact/Contact.h
		Includes/Contact/ContactDetection.h	
		Includes/Contact/Elongation.h
		Includes/Contact/ComputingForce.h

		Includes/Dynamic/Move.h
		Includes/Dynamic/Compaction.h
		Includes/Dynamic/PowderPaQ.h
		Includes/Dynamic/Evolution.h
			
		Includes/Repository/ReadWrite.h
)

set(SandPanda_FILES
		${INCLUDES_FILES}
		${SOURCES_FILES}
)

set (CMAKE_CXX_STANDARD 20)

add_executable(
		SandPanda
		${SandPanda_FILES}
)

target_link_libraries(
		SandPanda
		${GTEST_LIBRARIES}
)


if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "-O0 -g")
else()
    set(CMAKE_CXX_FLAGS "-O3")
endif()

enable_testing()

add_subdirectory("Tests")
