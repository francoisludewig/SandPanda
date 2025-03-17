#include <gtest/gtest.h>
#include "../Includes/Object/MechanicalPoint.h"

class MechanicalPointTest : public ::testing::Test {
protected:
    void SetUp() override {
        point = new MechanicalPoint();
    }

    void TearDown() override {
        delete point;
    }

    MechanicalPoint* point;
};

// Test du constructeur par défaut
TEST_F(MechanicalPointTest, DefaultConstructor) {
    EXPECT_DOUBLE_EQ(point->x, 0.0);
    EXPECT_DOUBLE_EQ(point->y, 0.0);
    EXPECT_DOUBLE_EQ(point->z, 0.0);

    EXPECT_DOUBLE_EQ(point->q0, 1.0);
    EXPECT_DOUBLE_EQ(point->q1, 0.0);
    EXPECT_DOUBLE_EQ(point->q2, 0.0);
    EXPECT_DOUBLE_EQ(point->q3, 0.0);

    EXPECT_DOUBLE_EQ(point->vx, 0.0);
    EXPECT_DOUBLE_EQ(point->vy, 0.0);
    EXPECT_DOUBLE_EQ(point->vz, 0.0);

    EXPECT_DOUBLE_EQ(point->wx, 0.0);
    EXPECT_DOUBLE_EQ(point->wy, 0.0);
    EXPECT_DOUBLE_EQ(point->wz, 0.0);

    EXPECT_DOUBLE_EQ(point->Fx, 0.0);
    EXPECT_DOUBLE_EQ(point->Fy, 0.0);
    EXPECT_DOUBLE_EQ(point->Fz, 0.0);

    EXPECT_DOUBLE_EQ(point->Mx, 0.0);
    EXPECT_DOUBLE_EQ(point->My, 0.0);
    EXPECT_DOUBLE_EQ(point->Mz, 0.0);
}

// Test du constructeur par copie
TEST_F(MechanicalPointTest, CopyConstructor) {
    MechanicalPoint other;
    other.x = 1.0;
    other.y = 2.0;
    other.z = 3.0;
    other.vx = 4.0;
    other.vy = 5.0;
    other.vz = 6.0;

    MechanicalPoint copy(other);

    EXPECT_DOUBLE_EQ(copy.x, 1.0);
    EXPECT_DOUBLE_EQ(copy.y, 2.0);
    EXPECT_DOUBLE_EQ(copy.z, 3.0);
    EXPECT_DOUBLE_EQ(copy.vx, 4.0);
    EXPECT_DOUBLE_EQ(copy.vy, 5.0);
    EXPECT_DOUBLE_EQ(copy.vz, 6.0);
}

// Test de l'opérateur d'affectation par copie
TEST_F(MechanicalPointTest, CopyAssignment) {
    MechanicalPoint other;
    other.x = 7.0;
    other.y = 8.0;
    other.z = 9.0;

    *point = other;

    EXPECT_DOUBLE_EQ(point->x, 7.0);
    EXPECT_DOUBLE_EQ(point->y, 8.0);
    EXPECT_DOUBLE_EQ(point->z, 9.0);
}

// Test du constructeur par déplacement
TEST_F(MechanicalPointTest, MoveConstructor) {
    MechanicalPoint other;
    other.x = 10.0;
    other.y = 11.0;
    other.z = 12.0;

    MechanicalPoint moved(std::move(other));

    EXPECT_DOUBLE_EQ(moved.x, 10.0);
    EXPECT_DOUBLE_EQ(moved.y, 11.0);
    EXPECT_DOUBLE_EQ(moved.z, 12.0);
}

// Test de l'opérateur d'affectation par déplacement
TEST_F(MechanicalPointTest, MoveAssignment) {
    MechanicalPoint other;
    other.x = 13.0;
    other.y = 14.0;
    other.z = 15.0;

    *point = std::move(other);

    EXPECT_DOUBLE_EQ(point->x, 13.0);
    EXPECT_DOUBLE_EQ(point->y, 14.0);
    EXPECT_DOUBLE_EQ(point->z, 15.0);
}
