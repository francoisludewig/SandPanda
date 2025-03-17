//
// Created by ludfr on 14-03-25.
//
#include <gtest/gtest.h>
#include <cmath>
#include "Object/Sphere.h"
#include "Contact/Elongation.h"


class SphereTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
    Sphere sphere;
};

TEST_F(SphereTest, DefaultConstructor) {
    EXPECT_NEAR(sphere.Radius(), 0.0005, 1e-6);
    EXPECT_NEAR(sphere.Mass(), (4. / 3.) * 2500 * M_PI * pow(0.0005, 3), 1e-6);
    EXPECT_EQ(sphere.NoBodies(), 1);
}

TEST_F(SphereTest, Radius) {
    sphere.setRadius(2.0);
    EXPECT_NEAR(sphere.Radius(), 0.001, 1e-6);
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
    sphere.CancelVelocity();
    EXPECT_NEAR(sphere.getFx(), 0.0, 1e-6);
    EXPECT_NEAR(sphere.getFy(), 0.0, 1e-6);
    EXPECT_NEAR(sphere.getFz(), 0.0, 1e-6);
}

TEST_F(SphereTest, RandomVelocity) {
    sphere.RandomVelocity(5.0, 2.0);
    double vNorm = sqrt(pow(sphere.vx, 2) + pow(sphere.vy, 2) + pow(sphere.vz, 2));
    EXPECT_NEAR(vNorm, 5.0, 1e-6);
}

TEST_F(SphereTest, ComputeCTD) {
    sphere.ComputeCTD(1.0, 2.0, M_PI);
    EXPECT_NEAR(sphere.getFx(), sphere.Mass() * 1.0 * 4.0 * cos(2.0 * M_PI), 1e-6);
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