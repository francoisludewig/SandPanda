//
// Created by ludfr on 14-03-25.
//
#include <gtest/gtest.h>
#include <cmath>
#include "Object/Sphere.h"
#include "Contact/Elongation.h"


class SphereTest : public ::testing::Test {
protected:
    void SetUp() override {
        sphere.x = 1.0;
        sphere.y = 2.0;
        sphere.z = 3.0;
        sphere.vx = 0.5;
        sphere.vy = -0.2;
        sphere.vz = 0.1;
        sphere.wx = 0.05;
        sphere.wy = -0.03;
        sphere.wz = 0.02;
        sphere.Fx = 10.0;
        sphere.Fy = -5.0;
        sphere.Fz = 2.0;
        sphere.Mx = 1.0;
        sphere.My = -0.5;
        sphere.Mz = 0.2;
    }

    void TearDown() override {}

    Sphere sphere;
};

// Test de la méthode resetForceAndMomentum
TEST_F(SphereTest, InitTimeStepResetsForcesAndMoments) {
    sphere.resetForceAndMomentum();

    EXPECT_DOUBLE_EQ(sphere.Fx, 0.0);
    EXPECT_DOUBLE_EQ(sphere.Fy, 0.0);
    EXPECT_DOUBLE_EQ(sphere.Fz, 0.0);
    EXPECT_DOUBLE_EQ(sphere.Mx, 0.0);
    EXPECT_DOUBLE_EQ(sphere.My, 0.0);
    EXPECT_DOUBLE_EQ(sphere.Mz, 0.0);
}

// Test de la méthode move avec autoIntegrate = true
TEST_F(SphereTest, MoveUpdatesPositionWhenAutoIntegrateIsTrue) {
    double dt = 0.1;
    sphere.autoIntegrate = true;

    double xBefore = sphere.x;
    double yBefore = sphere.y;
    double zBefore = sphere.z;

    sphere.move(dt);

    EXPECT_NE(sphere.x, xBefore);  // La position doit changer
    EXPECT_NE(sphere.y, yBefore);
    EXPECT_NE(sphere.z, zBefore);
}

// Test de move avec autoIntegrate = false
TEST_F(SphereTest, MoveDoesNothingWhenAutoIntegrateIsFalse) {
    double dt = 0.1;
    sphere.autoIntegrate = false;

    double xBefore = sphere.x;
    double yBefore = sphere.y;
    double zBefore = sphere.z;

    sphere.move(dt);

    EXPECT_DOUBLE_EQ(sphere.x, xBefore);
    EXPECT_DOUBLE_EQ(sphere.y, yBefore);
    EXPECT_DOUBLE_EQ(sphere.z, zBefore);
}

TEST_F(SphereTest, DefaultConstructor) {
    EXPECT_NEAR(sphere.Radius(), 0.0005, 1e-6);
    EXPECT_NEAR(sphere.Mass(), (4. / 3.) * 2500 * M_PI * pow(0.0005, 3), 1e-6);
    EXPECT_EQ(sphere.NoBodies(), 1);
}

TEST_F(SphereTest, Radius) {
    sphere.setRadius(2.0);
    EXPECT_NEAR(sphere.Radius(), 0.001, 1e-6);
}


TEST_F(SphereTest, Move) {

}


TEST_F(SphereTest, MassCalculation) {
    double initialMass = sphere.Mass();
    sphere.setRadius(2.0);
    EXPECT_GT(sphere.Mass(), initialMass);
}

TEST_F(SphereTest, CountTest) {
    EXPECT_EQ(sphere.count(), 0);
}

TEST_F(SphereTest, CancelVelocity) {
    sphere.resetVelocities();
    EXPECT_NEAR(sphere.vx, 0.0, 1e-6);
    EXPECT_NEAR(sphere.vy, 0.0, 1e-6);
    EXPECT_NEAR(sphere.vz, 0.0, 1e-6);
}

TEST_F(SphereTest, RandomVelocity) {
    sphere.RandomVelocity(5.0, 2.0);
    double vNorm = sqrt(pow(sphere.vx, 2) + pow(sphere.vy, 2) + pow(sphere.vz, 2));
    EXPECT_NEAR(vNorm, 5.0, 1e-6);
}

TEST_F(SphereTest, HollowBallCheck) {
    sphere.setIsHollowBall(true);
    EXPECT_EQ(sphere.NoAvatar(), 0);
}

TEST_F(SphereTest, InitXsi) {
    sphere.InitXsi();
    EXPECT_EQ(sphere.Nneighbour, 0);
}

TEST_F(SphereTest, AddXsi) {
    Elongation elong;
    sphere.AddXsi(elong, 1, 2, 3);
    EXPECT_EQ(sphere.Nneighbour2, 1);
    EXPECT_EQ(sphere.Nneighbour, 0);
}

TEST_F(SphereTest, AddXsiAndInitXsi) {
    Elongation elong;
    sphere.AddXsi(elong, 1, 2, 3);
    EXPECT_EQ(sphere.Nneighbour2, 1);
    EXPECT_EQ(sphere.Nneighbour, 0);
    sphere.InitXsi();
    EXPECT_EQ(sphere.Nneighbour2, 0);
    EXPECT_EQ(sphere.Nneighbour, 1);
}

TEST_F(SphereTest, FoundIt) {
    Elongation elong;
    elong.x = 0.00000125896;
    elong.y = 0.00000006548;
    elong.z = 0.00000000112321;
    sphere.AddXsi(elong, 1, 2, 3);
    sphere.InitXsi();
    Elongation found = sphere.FoundIt(1, 2, 3);
    EXPECT_EQ(found.x, elong.x);
    EXPECT_EQ(found.y, elong.y);
    EXPECT_EQ(found.z, elong.z);
    EXPECT_EQ(found.status, elong.status);
}

TEST_F(SphereTest, FoundItWhenDoesntExist) {
    sphere.InitXsi();
    Elongation found = sphere.FoundIt(1, 2, 3);
    EXPECT_EQ(found.x, 0);
    EXPECT_EQ(found.y, 0);
    EXPECT_EQ(found.z, 0);
    EXPECT_EQ(found.status, 0);
}