//
// Created by ludfr on 14-03-25.
//
#include <gtest/gtest.h>
#include <cmath>
#include "Object/Sinusoid.h"


class SinusoidTest : public ::testing::Test {
protected:
    virtual void SetUp() {
    }

    virtual void TearDown() {}
};

TEST_F(SinusoidTest, DefaultConstructor) {
    Sinusoid sinusoid;
    EXPECT_EQ(0, sinusoid.Value(0));
    EXPECT_EQ(0, sinusoid.Value(1));
}

TEST_F(SinusoidTest, ConstantConstructor) {
    Sinusoid sinusoid(3);
    EXPECT_EQ(3, sinusoid.Value(0));
    EXPECT_EQ(3, sinusoid.Value(1));
}

TEST_F(SinusoidTest, OscillantConstructor) {
    Sinusoid sinusoid(0, 1, 2*M_PI, 0);
    EXPECT_GE(pow(10,-15), abs(sinusoid.Value(0)));
    EXPECT_EQ(1, sinusoid.Value(0.25));
    EXPECT_GE(pow(10,-15), abs(sinusoid.Value(0.5)));
    EXPECT_EQ(-1, sinusoid.Value(0.75));
    EXPECT_GE(pow(10,-15), abs(sinusoid.Value(1)));
}

TEST_F(SinusoidTest, MaxWhenNull) {
    Sinusoid sinusoid;
    EXPECT_EQ(0, sinusoid.Max());
}

TEST_F(SinusoidTest, MaxWhenConstant) {
    Sinusoid sinusoid(3);
    EXPECT_EQ(3, sinusoid.Max());
}

TEST_F(SinusoidTest, MaxWhenOscillant) {
    Sinusoid sinusoid(0, 1, 2*M_PI, 0);
    EXPECT_EQ(1, sinusoid.Max());
}

TEST_F(SinusoidTest, MaxWhenGeneral) {
    Sinusoid sinusoid(3, 1, 2*M_PI, 0);
    EXPECT_EQ(4, sinusoid.Max());
}

TEST_F(SinusoidTest, DelayTrWhenNull) {
    Sinusoid sinusoid;
    EXPECT_EQ(0, sinusoid.DelayTr());
}

TEST_F(SinusoidTest, DelayTrWhenConstant) {
    Sinusoid sinusoid(3);
    EXPECT_EQ(10000, sinusoid.DelayTr());
}

TEST_F(SinusoidTest, DelayTrWhenOScillant) {
    Sinusoid sinusoid(0, 1, 2*M_PI, 0);
    EXPECT_EQ(1, sinusoid.DelayTr());
}

TEST_F(SinusoidTest, DelayTrWhenGeneral) {
    // TO BE 10000 ???
    Sinusoid sinusoid(3, 1, 2*M_PI, 0);
    EXPECT_EQ(1, sinusoid.DelayTr());
}

TEST_F(SinusoidTest, DelayRotWhenNull) {
    Sinusoid sinusoid;
    EXPECT_EQ(0, sinusoid.DelayRot());
}

TEST_F(SinusoidTest, DelayRotWhenConstant) {
    Sinusoid sinusoid(3);
    EXPECT_EQ(2*M_PI/3, sinusoid.DelayRot());
}

TEST_F(SinusoidTest, DelayRotWhenOScillant) {
    Sinusoid sinusoid(0, 1, 2*M_PI, 0);
    EXPECT_EQ(2*M_PI, sinusoid.DelayRot());
}

TEST_F(SinusoidTest, DelayRotWhenGeneral) {
    // TO BE 10000 ???
    Sinusoid sinusoid(3, 1, 2*M_PI, 0);
    EXPECT_EQ(M_PI_2, sinusoid.DelayRot());
}