cmake_minimum_required(VERSION 3.6)

project(SandPanda VERSION 1.0.0)

set(SOURCES_FILES
 #Sources/AffinityCache.cpp
 Sources/HollowBall.cpp
 Sources/Body.cpp
 Sources/MasterSolid.cpp
 Sources/BodySpecie.cpp		
 Sources/Move.cpp
 Sources/Option.cpp
 Sources/Compaction.cpp		
 Sources/Periodicity.cpp
 Sources/ComputingForce.cpp	
 Sources/Plan.cpp
 Sources/Cone.cpp		
 Sources/PlanR.cpp
 Sources/Contact.cpp		
 Sources/PowderPaQ.cpp
 Sources/ContactDetection.cpp	
 Sources/ReadWrite.cpp
 Sources/Data.cpp
 Sources/Elbow.cpp		
 Sources/Solid.cpp
 Sources/Elongation.cpp		
 Sources/Sphere.cpp
 Sources/Evolution.cpp
 Sources/Gravity.cpp	
 Sources/Velocity.cpp	
 Sources/Sinusoid.cpp	
 Sources/main.cpp
        Sources/Monitoring.cpp
)

set(INCLUDES_FILES
 #Includes/AffinityCache.h
 Includes/HollowBall.h
 Includes/Body.h
 Includes/MasterSolid.h
 Includes/BodySpecie.h		
 Includes/Move.h
 Includes/Compaction.h		
 Includes/Option.h
 Includes/ComputingForce.h	
 Includes/Periodicity.h
 Includes/Cone.h			
 Includes/Plan.h
 Includes/Contact.h		
 Includes/PlanR.h
 Includes/ContactDetection.h	
 Includes/PowderPaQ.h
 Includes/Data.h			
 Includes/ReadWrite.h
 Includes/Elbow.h
 Includes/Elongation.h		
 Includes/Solid.h
 Includes/Evolution.h		
 Includes/Sphere.h		
 Includes/Gravity.h	
 Includes/Velocity.h
 Includes/Sinusoid.h
        Includes/Monitoring.h
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

#add_subdirectory("Tests")
