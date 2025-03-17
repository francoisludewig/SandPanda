#include <gtest/gtest.h>
#include "../Includes/Object/PlanR.h"

class PlanRTest : public ::testing::Test {
protected:
    void SetUp() override {
        planR = new PlanR();
    }

    void TearDown() override {
        delete planR;
    }

    PlanR* planR;
};

// Test du constructeur par défaut
TEST_F(PlanRTest, DefaultConstructor) {
    EXPECT_DOUBLE_EQ(planR->r, 0.5);
    EXPECT_DOUBLE_EQ(planR->dn, 0.0);
    EXPECT_EQ(planR->periodic, -9);
}

// Test de lecture et d'écriture dans un fichier (Mock)
TEST_F(PlanRTest, ReadWriteToFile) {
    FILE* file = tmpfile();
    ASSERT_NE(file, nullptr);

    planR->dn = 1.23;
    planR->r = 2.34;
    planR->periodic = 1;

    planR->writeToFile(file);

    rewind(file);

    PlanR newPlanR;
    newPlanR.readFromFile(file);

    EXPECT_DOUBLE_EQ(newPlanR.dn, 1.23);
    EXPECT_DOUBLE_EQ(newPlanR.r, 2.34);
    EXPECT_EQ(newPlanR.periodic, 1);

    fclose(file);
}

// Test de l'écriture binaire
TEST_F(PlanRTest, WriteOutFile) {
    FILE* file = tmpfile();
    ASSERT_NE(file, nullptr);

    planR->dn = 1.5;
    planR->r = 3.0;
    planR->periodic = 1;

    planR->writeToFile(file);

    rewind(file);

    PlanR newPlanR;
    newPlanR.readFromFile(file);

    EXPECT_DOUBLE_EQ(newPlanR.dn, 1.5);
    EXPECT_DOUBLE_EQ(newPlanR.r, 3.0);
    EXPECT_EQ(newPlanR.periodic, 1);

    fclose(file);
}
