#include <gtest/gtest.h>
#include "../Includes/Object/Elbow.h"

class ElbowTest : public ::testing::Test {
protected:
    void SetUp() override {
        elbow = new Elbow();
    }

    void TearDown() override {
        delete elbow;
    }

    Elbow* elbow;
};

// Test du constructeur par défaut
TEST_F(ElbowTest, DefaultConstructor) {
    EXPECT_DOUBLE_EQ(elbow->x, 0.0);
    EXPECT_DOUBLE_EQ(elbow->y, 0.0);
    EXPECT_DOUBLE_EQ(elbow->z, 0.0);
    EXPECT_DOUBLE_EQ(elbow->nx, 1.0);
    EXPECT_DOUBLE_EQ(elbow->ny, 0.0);
    EXPECT_DOUBLE_EQ(elbow->nz, 0.0);
    EXPECT_DOUBLE_EQ(elbow->tx, 0.0);
    EXPECT_DOUBLE_EQ(elbow->ty, 1.0);
    EXPECT_DOUBLE_EQ(elbow->tz, 0.0);
    EXPECT_DOUBLE_EQ(elbow->sx, 0.0);
    EXPECT_DOUBLE_EQ(elbow->sy, 0.0);
    EXPECT_DOUBLE_EQ(elbow->sz, 1.0);
    EXPECT_DOUBLE_EQ(elbow->vx, 0.0);
    EXPECT_DOUBLE_EQ(elbow->vy, 0.0);
    EXPECT_DOUBLE_EQ(elbow->vz, 0.0);
    EXPECT_DOUBLE_EQ(elbow->wx, 0.0);
    EXPECT_DOUBLE_EQ(elbow->wy, 0.0);
    EXPECT_DOUBLE_EQ(elbow->wz, 0.0);
    EXPECT_DOUBLE_EQ(elbow->r, 0.1);
    EXPECT_DOUBLE_EQ(elbow->xi, -0.25);
    EXPECT_DOUBLE_EQ(elbow->yi, 0.0);
    EXPECT_DOUBLE_EQ(elbow->zi, 0.0);
    EXPECT_DOUBLE_EQ(elbow->xf, 0.0);
    EXPECT_DOUBLE_EQ(elbow->yf, 0.0);
    EXPECT_DOUBLE_EQ(elbow->zf, 0.25);
    EXPECT_DOUBLE_EQ(elbow->cx, 0.0);
    EXPECT_DOUBLE_EQ(elbow->cy, 0.0);
    EXPECT_DOUBLE_EQ(elbow->cz, 1.0);
}

// Test de lecture et d'écriture dans un fichier
TEST_F(ElbowTest, ReadWriteToFile) {
    FILE* file = tmpfile();
    ASSERT_NE(file, nullptr);

    elbow->xi = 1.1;
    elbow->yi = 2.2;
    elbow->zi = 3.3;
    elbow->xf = 4.4;
    elbow->yf = 5.5;
    elbow->zf = 6.6;
    elbow->x = 7.7;
    elbow->y = 8.8;
    elbow->z = 9.9;
    elbow->R = 10.1;
    elbow->alpha = 11.1;
    elbow->r = 12.1;

    elbow->WriteToFile(file);
    rewind(file);

    Elbow newElbow;
    newElbow.ReadFromFile(file);

    EXPECT_DOUBLE_EQ(newElbow.xi, 1.1);
    EXPECT_DOUBLE_EQ(newElbow.yi, 2.2);
    EXPECT_DOUBLE_EQ(newElbow.zi, 3.3);
    EXPECT_DOUBLE_EQ(newElbow.xf, 4.4);
    EXPECT_DOUBLE_EQ(newElbow.yf, 5.5);
    EXPECT_DOUBLE_EQ(newElbow.zf, 6.6);
    EXPECT_DOUBLE_EQ(newElbow.x, 7.7);
    EXPECT_DOUBLE_EQ(newElbow.y, 8.8);
    EXPECT_DOUBLE_EQ(newElbow.z, 9.9);
    EXPECT_DOUBLE_EQ(newElbow.R, 10.1);
    EXPECT_DOUBLE_EQ(newElbow.alpha, 11.1);
    EXPECT_DOUBLE_EQ(newElbow.r, 12.1);

    fclose(file);
}

// Test de l'écriture binaire
TEST_F(ElbowTest, WriteOutFileBinary) {
    FILE* file = tmpfile();
    ASSERT_NE(file, nullptr);

    elbow->xi = 1.5;
    elbow->yi = 2.5;
    elbow->zi = 3.5;
    elbow->xf = 4.5;
    elbow->yf = 5.5;
    elbow->zf = 6.5;
    elbow->R = 7.5;
    elbow->alpha = 8.5;
    elbow->r = 9.5;

    elbow->WriteToFile(file);
    rewind(file);

    Elbow newElbow;
    newElbow.ReadFromFile(file);

    EXPECT_DOUBLE_EQ(newElbow.xi, 1.5);
    EXPECT_DOUBLE_EQ(newElbow.yi, 2.5);
    EXPECT_DOUBLE_EQ(newElbow.zi, 3.5);
    EXPECT_DOUBLE_EQ(newElbow.xf, 4.5);
    EXPECT_DOUBLE_EQ(newElbow.yf, 5.5);
    EXPECT_DOUBLE_EQ(newElbow.zf, 6.5);
    EXPECT_DOUBLE_EQ(newElbow.R, 7.5);
    EXPECT_DOUBLE_EQ(newElbow.alpha, 8.5);
    EXPECT_DOUBLE_EQ(newElbow.r, 9.5);

    fclose(file);
}

// Test du déplacement de l'Elbow
TEST_F(ElbowTest, Move) {
    elbow->x = 1.0;
    elbow->y = 2.0;
    elbow->z = 3.0;
    elbow->vx = 1.0;
    elbow->vy = 2.0;
    elbow->vz = 3.0;

    elbow->Move(1.0, 1.0);

    EXPECT_NEAR(elbow->x, 1.0 + elbow->vx * 1.0, 1e-6);
    EXPECT_NEAR(elbow->y, 2.0 + elbow->vy * 1.0, 1e-6);
    EXPECT_NEAR(elbow->z, 3.0 + elbow->vz * 1.0, 1e-6);
}

// Test des fonctions Vmax, Wmax et Delay
TEST_F(ElbowTest, VelocityFunctions) {
    EXPECT_DOUBLE_EQ(elbow->Vmax(), elbow->V.VMax());
    EXPECT_DOUBLE_EQ(elbow->Wmax(), elbow->V.WMax());
    EXPECT_DOUBLE_EQ(elbow->Delay(), elbow->V.Delay());
}
