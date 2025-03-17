//
// Created by ludfr on 14-03-25.
//
#include <gtest/gtest.h>
#include <cmath>
#include "Object/Velocity.h"


class VelocityTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(VelocityTest, DefaultConstructor){
    Velocity velocity;
    EXPECT_EQ(0, velocity.ox);
    EXPECT_EQ(0, velocity.oy);
    EXPECT_EQ(0, velocity.oz);

    EXPECT_EQ(0, velocity.ValueOfVx(10));
    EXPECT_EQ(0, velocity.ValueOfVy(10));
    EXPECT_EQ(0, velocity.ValueOfVz(10));
    EXPECT_EQ(0, velocity.ValueOfWx(10));
    EXPECT_EQ(0, velocity.ValueOfWy(10));
    EXPECT_EQ(0, velocity.ValueOfWz(10));
}
