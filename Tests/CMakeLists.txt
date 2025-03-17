cmake_minimum_required(VERSION 3.6)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest
        GIT_TAG v1.14.0
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(SOURCES_FILES
        Object/TestVelocity.cpp
        Object/TestSinusoid.cpp
        Object/TestSphere.cpp
        Object/TestSolid.cpp
        Object/TestPlan.cpp
        Object/TestPlanR.cpp
        Object/TestElbow.cpp
        Object/TestBody.cpp
        Object/TestBodySpecie.cpp
        Object/TestMechanicalPoint.cpp
        Object/TestHollowBall.cpp
        main.cpp
)

set(FILES
        ${SOURCES_FILES}
)

add_executable(
        SandPanda.Tests
        ${FILES}
        ${SandPanda_SOURCE_DIR}/Sources/Object/Sinusoid.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Object/Velocity.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Object/Sphere.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Object/Solid.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Object/Plan.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Object/PlanR.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Object/Elbow.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Object/HollowBall.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Object/Body.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Object/BodySpecie.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Configuration/Gravity.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Contact/Elongation.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Contact/Contact.cpp
        ${SandPanda_SOURCE_DIR}/Sources/Contact/ContactDetection.cpp
        ${tests_files}
)

enable_testing()

target_link_libraries(
        SandPanda.Tests
        gtest_main
        gmock_main
)

include_directories (${SandPanda_SOURCE_DIR}/Includes)

include(GoogleTest)
gtest_discover_tests(SandPanda.Tests)