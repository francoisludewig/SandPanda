/**
 * @file TestBody.cpp
 * @brief Unit tests for the Body class.
 */

#include <gtest/gtest.h>
#include <vector>
#include "../Includes/Object/Body.h"
#include "../Includes/Object/Sphere.h"
#include "../Includes/Object/BodySpecie.h"
#include "../Includes/Configuration/Gravity.h"
#include "../Includes/Contact/Elongation.h"

/**
 * @brief Tests for the Body class.
 */
class BodyTest : public ::testing::Test {
protected:
    void SetUp() override {
        body = new Body();
    }

    void TearDown() override {
        delete body;
    }

    Body *body;
};

/**
 * @brief Test the default constructor.
 */
TEST_F(BodyTest, ConstructorInitializesCorrectly) {
    EXPECT_EQ(body->Ng, 0);
    EXPECT_EQ(body->Rmax, 0);
    EXPECT_EQ(body->m, 0);
    EXPECT_EQ(body->sp, 0);
    EXPECT_EQ(body->ActiveRotation, 0);

    for (int i = 0; i < 250; ++i) {
        EXPECT_EQ(body->NumNeighbour[i], -9);
        EXPECT_EQ(body->type[i], -1);
        EXPECT_EQ(body->NumFromBody[i], -9);
        EXPECT_EQ(body->SelfNumFromBody[i], -9);
    }

    EXPECT_EQ(body->Nneighbour, 0);
    EXPECT_EQ(body->Nneighbour2, 50);
}

/**
 * @brief Test the LoadFromFile method with a mock file.
 */
TEST_F(BodyTest, LoadFromFile) {
    FILE *ft = fopen("test_input.txt", "w+");
    fprintf(ft, "1 2\n0.5 0.6 0.7\n1.0 0.0 0.0 0.0\n0.1 0.2 0.3\n0.01 0.02 0.03\n");
    fclose(ft);

    ft = fopen("test_input.txt", "r");
    ASSERT_NE(ft, nullptr);
    body->LoadFromFile(ft);
    fclose(ft);

    EXPECT_EQ(body->sp, 1);
    EXPECT_EQ(body->NhollowBall, 2);
}

/**
 * @brief Test the TimeStepInitialization method.
 */
TEST_F(BodyTest, TimeStepInitialization) {
    body->TimeStepInitialization();
    EXPECT_EQ(body->Fx, 0);
    EXPECT_EQ(body->Fy, 0);
    EXPECT_EQ(body->Fz, 0);
    EXPECT_EQ(body->Mx, 0);
    EXPECT_EQ(body->My, 0);
    EXPECT_EQ(body->Mz, 0);
}

/**
 * @brief Test the move method.
 */
TEST_F(BodyTest, MoveUpdatesPosition) {
    body->vx = 1.0;
    body->vy = 2.0;
    body->vz = 3.0;

    body->Move(1.0);

    EXPECT_DOUBLE_EQ(body->x, 1.0);
    EXPECT_DOUBLE_EQ(body->y, 2.0);
    EXPECT_DOUBLE_EQ(body->z, 3.0);
}

/**
 * @brief Test UpDateVelocity method.
 */
TEST_F(BodyTest, UpDateVelocity) {
    Gravity gravity;
    gravity.ngx = 0.0;
    gravity.ngy = -9.81;
    gravity.ngz = 0.0;
    gravity.G = 1.0;

    body->m = 10.0;
    body->UpDateVelocity(1.0, gravity);

    EXPECT_DOUBLE_EQ(body->Fy, 10.0 * -9.81);
}

/**
 * @brief Test the resetVelocities method.
 */
TEST_F(BodyTest, CancelVelocity) {
    body->vx = 5.0;
    body->vy = 3.0;
    body->vz = -1.0;
    body->wx = 0.5;
    body->wy = -0.3;
    body->wz = 0.2;

    body->CancelVelocity();

    EXPECT_DOUBLE_EQ(body->vx, 0);
    EXPECT_DOUBLE_EQ(body->vy, 0);
    EXPECT_DOUBLE_EQ(body->vz, 0);
    EXPECT_DOUBLE_EQ(body->wx, 0);
    EXPECT_DOUBLE_EQ(body->wy, 0);
    EXPECT_DOUBLE_EQ(body->wz, 0);
}

/**
 * @brief Test the NumberOfSphere method.
 */
TEST_F(BodyTest, NumberOfSphere) {
    body->Ng = 5;
    EXPECT_EQ(body->NumberOfSphere(), 5);
}

/**
 * @brief Test the Num method.
 */
TEST_F(BodyTest, Num) {
    body->numl = 42;
    EXPECT_EQ(body->Num(), 42);
}

/**
 * @brief Test the GetRmax method.
 */
TEST_F(BodyTest, GetRmax) {
    body->Rmax = 12.34;
    EXPECT_DOUBLE_EQ(body->GetRmax(), 12.34);
}

/**
 * @brief Test the SetActiveRotation method.
 */
TEST_F(BodyTest, SetActiveRotation) {
    body->SetActiveRotation(1);
    EXPECT_EQ(body->ActiveRotation, 1);

    body->SetActiveRotation(0);
    EXPECT_EQ(body->ActiveRotation, 0);

    body->SetActiveRotation(2);
    EXPECT_NE(body->ActiveRotation, 2); // Should remain unchanged
}

/**
 * @brief Test the AddXsi method.
 */
TEST_F(BodyTest, AddXsi) {
    Elongation e;
    e.x = 0.5;
    e.y = -0.5;
    e.z = 1.0;
    body->AddXsi(e, 10, 2, 5, 8);

    EXPECT_EQ(body->NumNeighbour[body->Nneighbour2 - 1], 10);
    EXPECT_EQ(body->type[body->Nneighbour2 - 1], 2);
    EXPECT_EQ(body->SelfNumFromBody[body->Nneighbour2 - 1], 5);
    EXPECT_EQ(body->NumFromBody[body->Nneighbour2 - 1], 8);
    EXPECT_DOUBLE_EQ(body->xsi[body->Nneighbour2 - 1].x, 0.5);
    EXPECT_DOUBLE_EQ(body->xsi[body->Nneighbour2 - 1].y, -0.5);
    EXPECT_DOUBLE_EQ(body->xsi[body->Nneighbour2 - 1].z, 1.0);
}

/**
 * @brief Test the FoundIt method.
 */
TEST_F(BodyTest, FoundIt) {
    Elongation e;
    e.x = 1.1;
    e.y = 2.2;
    e.z = 3.3;
    body->AddXsi(e, 20, 3, 6, 9);
    body->InitXsi();
    Elongation found = body->FoundIt(20, 3, 6, 9);

    EXPECT_DOUBLE_EQ(found.x, 1.1);
    EXPECT_DOUBLE_EQ(found.y, 2.2);
    EXPECT_DOUBLE_EQ(found.z, 3.3);
}

/**
 * @brief Test edge case of FoundIt when no match is found.
 */
TEST_F(BodyTest, FoundItNoMatch) {
    Elongation found = body->FoundIt(99, 99, 99, 99);
    EXPECT_DOUBLE_EQ(found.x, 0);
    EXPECT_DOUBLE_EQ(found.y, 0);
    EXPECT_DOUBLE_EQ(found.z, 0);
}
