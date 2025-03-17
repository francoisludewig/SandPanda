//
// Created by ludfr on 15-03-25.
//
#include <gtest/gtest.h>
#include "Object/Solid.h"
#include "Configuration/Gravity.h"

class SolidTest : public ::testing::Test {
protected:
    void SetUp() override {
        solid = new Solid();
    }

    void TearDown() override {
        delete solid;
    }

    Solid *solid = nullptr;
};

// Test default constructor
TEST_F(SolidTest, DefaultConstructor) {
    EXPECT_EQ(solid->nx, 1);
    EXPECT_EQ(solid->ny, 0);
    EXPECT_EQ(solid->nz, 0);
    EXPECT_EQ(solid->tx, 0);
    EXPECT_EQ(solid->ty, 1);
    EXPECT_EQ(solid->tz, 0);
    EXPECT_EQ(solid->sx, 0);
    EXPECT_EQ(solid->sy, 0);
    EXPECT_EQ(solid->sz, 1);
    EXPECT_EQ(solid->Mass, 1);
    EXPECT_EQ(solid->In, 1);
    EXPECT_EQ(solid->It, 1);
    EXPECT_EQ(solid->Is, 1);
    EXPECT_EQ(solid->activeGravity, 0);
    EXPECT_EQ(solid->Force, 0);
    EXPECT_EQ(solid->NCell, 0);
    EXPECT_EQ(solid->Cell, nullptr);
}

// Test gravity activation
TEST_F(SolidTest, GravityActivation) {
    solid->OnOffGravity(true);
    EXPECT_EQ(solid->activeGravity, 1);

    solid->OnOffGravity(false);
    EXPECT_EQ(solid->activeGravity, 0);
}

// Test velocity reset
TEST_F(SolidTest, SetVelocityToZero) {
    solid->vx = 5.0;
    solid->vy = 3.0;
    solid->vz = -2.0;

    solid->SetVelocityToZero();

    EXPECT_EQ(solid->vx, 0);
    EXPECT_EQ(solid->vy, 0);
    EXPECT_EQ(solid->vz, 0);
}

// Test setting and retrieving memory position
TEST_F(SolidTest, MemoryPosition) {
    solid->x = 1.2;
    solid->y = -3.4;
    solid->z = 5.6;

    solid->SetMemoryPosition();
    solid->x = 0;
    solid->y = 0;
    solid->z = 0;

    solid->GetMemoryPosition();

    EXPECT_EQ(solid->x, 1.2);
    EXPECT_EQ(solid->y, -3.4);
    EXPECT_EQ(solid->z, 5.6);
}

// Test force update logic
TEST_F(SolidTest, UpdateForce) {
    solid->Fcx = 0;
    solid->Fcy = 0;
    solid->Fcz = 0;
    solid->Mcx = 0;
    solid->Mcy = 0;
    solid->Mcz = 0;
    solid->activeGravity = 0;

    solid->UpDateForce();
    EXPECT_EQ(solid->Force, 0);

    solid->Fcx = 10.0;
    solid->UpDateForce();
    EXPECT_EQ(solid->Force, 1);
}

// Test velocity update under gravity
TEST_F(SolidTest, UpdateGravityVelocity) {
    Gravity gt;
    gt.ngx = 1.0;
    gt.ngy = 0.5;
    gt.ngz = -1.0;
    gt.G = 9.81;

    solid->vx = 0.0;
    solid->vy = 0.0;
    solid->vz = 0.0;
    solid->activeGravity = 1;
    solid->Mass = 2.0;

    solid->UpDateGravityVelocity(1.0, 0.1, gt);

    EXPECT_NEAR(solid->vx, (gt.ngx * gt.G) * 0.1, 1e-6);
    EXPECT_NEAR(solid->vy, (gt.ngy * gt.G) * 0.1, 1e-6);
    EXPECT_NEAR(solid->vz, (gt.ngz * gt.G) * 0.1, 1e-6);
}

// Test move function
TEST_F(SolidTest, Move) {
    solid->vx = 2.0;
    solid->vy = -1.0;
    solid->vz = 0.5;

    solid->x = 1.0;
    solid->y = 2.0;
    solid->z = 3.0;

    solid->Move(1.0);

    EXPECT_DOUBLE_EQ(solid->x, 3.0);
    EXPECT_DOUBLE_EQ(solid->y, 1.0);
    EXPECT_DOUBLE_EQ(solid->z, 3.5);
}
