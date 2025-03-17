#include <gtest/gtest.h>
#include <cmath>
#include "../Includes/Object/Plan.h"
#include "../Includes/Contact/Contact.h"
#include "../Includes/Object/Sphere.h"

class PlanTest : public ::testing::Test {
protected:
    void SetUp() override {
        plan = new Plan();
    }

    void TearDown() override {
        delete plan;
    }

    Plan* plan;
};

// Test du constructeur par défaut
TEST_F(PlanTest, DefaultConstructor) {
    EXPECT_EQ(plan->dt, 0.5);
    EXPECT_EQ(plan->ds, 0.5);
    EXPECT_EQ(plan->dn, 0.0);
    EXPECT_EQ(plan->periodic, -9);
    EXPECT_EQ(plan->sigma, 0.0);
    EXPECT_EQ(plan->ra, 0.0);
    EXPECT_EQ(plan->list, nullptr);
}

// Test de l'initialisation de la liste
TEST_F(PlanTest, InitList) {
    plan->InitList(10);
    EXPECT_EQ(plan->list, nullptr);  // La liste ne doit pas être allouée car periodic == -9

    plan->periodic = 1;
    plan->InitList(10);
    ASSERT_NE(plan->list, nullptr);
}

// Test de l'accès aux valeurs de `Ds` et `Dt`
TEST_F(PlanTest, GetDsDt) {
    EXPECT_DOUBLE_EQ(plan->Ds(), 0.5);
    EXPECT_DOUBLE_EQ(plan->Dt(), 0.5);
}

// Test de la mise à jour de l'angle Alpha
TEST_F(PlanTest, SetAlpha) {
    plan->SetAlpha(M_PI / 4);  // 45 degrés
    EXPECT_NEAR(plan->sigma, cos(M_PI / 4), 1e-6);
}

// Test de la normalisation des contacts
TEST_F(PlanTest, NormalCalculation) {
    Contact contact;
    Sphere sphere;
    sphere.ct_pl = 0;

    plan->SetAlpha(M_PI / 4);
    plan->Normal(&contact, &sphere);

    EXPECT_NEAR(sphere.ct_pl_nx, contact.nx, 1e-6);
    EXPECT_NEAR(sphere.ct_pl_ny, contact.ny, 1e-6);
    EXPECT_NEAR(sphere.ct_pl_nz, contact.nz, 1e-6);
    EXPECT_EQ(sphere.ct_pl, 1);
}