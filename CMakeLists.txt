cmake_minimum_required(VERSION 3.1.2)

project(SandPanda VERSION 1.0.0)

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

# Sources/AffinityCache.cpp
 Sources/Move.cpp
 Sources/Option.cpp
 Sources/Compaction.cpp		
 Sources/Periodicity.cpp
 Sources/ComputingForce.cpp
 Sources/PowderPaQ.cpp
 Sources/ReadWrite.cpp
 Sources/Configuration.cpp 
 Sources/Evolution.cpp
 Sources/MultiThread.cpp
 Sources/Gravity.cpp	
 Sources/SimulationDataManager.cpp	
 Sources/main.cpp
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
 Includes/Solids/SimulationData.h

 Includes/LinkedCells/LinkedCellFiller.h
 Includes/LinkedCells/SolidCellsBuilder.h	

 #Includes/AffinityCache.h
 Includes/Move.h
 Includes/Compaction.h		
 Includes/Option.h
 Includes/ComputingForce.h	
 Includes/Periodicity.h 
 Includes/PowderPaQ.h
 Includes/Configuration.h			
 Includes/ReadWrite.h 
 Includes/Evolution.h 
 Includes/MultiThread.h 
 Includes/Gravity.h	
 Includes/SimulationDataManager.h	
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
    set(CMAKE_CXX_FLAGS "-O3 -std=c++11")
endif()


enable_testing()

#add_subdirectory("Tests")
