/**
 * @file TestBodySpecie.cpp
 * @brief Unit tests for the BodySpecie class.
 */

#include <gtest/gtest.h>
#include "../Includes/Object/BodySpecie.h"
#include <cstdio>
#include <vector>

/**
 * @brief Tests for the BodySpecie class.
 */
class BodySpecieTest : public ::testing::Test {
protected:
    void SetUp() override {
        bodySpecie = new BodySpecie();
    }

    void TearDown() override {
        delete bodySpecie;
    }

    BodySpecie *bodySpecie;
};

/**
 * @brief Test the default constructor.
 */
TEST_F(BodySpecieTest, ConstructorInitializesCorrectly) {
    EXPECT_EQ(bodySpecie->Ng, 0);
    EXPECT_EQ(bodySpecie->m, 0);
    EXPECT_EQ(bodySpecie->sp, 0);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_DOUBLE_EQ(bodySpecie->Ine_1[i][j], 0);
        }
    }
}

/**
 * @brief Test the FreeMemory method.
 */
TEST_F(BodySpecieTest, FreeMemoryClearsVectors) {
    bodySpecie->num.push_back(1);
    bodySpecie->xl.push_back(0.1);
    bodySpecie->yl.push_back(0.2);
    bodySpecie->zl.push_back(0.3);
    bodySpecie->rl.push_back(0.4);

    bodySpecie->FreeMemory();

    EXPECT_TRUE(bodySpecie->num.empty());
    EXPECT_TRUE(bodySpecie->xl.empty());
    EXPECT_TRUE(bodySpecie->yl.empty());
    EXPECT_TRUE(bodySpecie->zl.empty());
    EXPECT_TRUE(bodySpecie->rl.empty());
}

/**
 * @brief Test the LoadFromFile method with a mock file.
 */
TEST_F(BodySpecieTest, LoadFromFile) {
    FILE *ft = fopen("test_body_specie.txt", "w+");
    fprintf(ft, "2\n0.5 0.6 0.7 0.8\n1.1 1.2 1.3 1.4\n10.0 5.5\n");
    fprintf(ft, "1.0 0.0 0.0\n0.0 1.0 0.0\n0.0 0.0 1.0\n");
    fclose(ft);

    ft = fopen("test_body_specie.txt", "r");
    ASSERT_NE(ft, nullptr);
    bodySpecie->LoadFromFile(ft);
    fclose(ft);

    EXPECT_EQ(bodySpecie->Ng, 2);
    EXPECT_EQ(bodySpecie->num.size(), 2);
    EXPECT_EQ(bodySpecie->xl.size(), 2);
    EXPECT_EQ(bodySpecie->yl.size(), 2);
    EXPECT_EQ(bodySpecie->zl.size(), 2);
    EXPECT_EQ(bodySpecie->rl.size(), 2);
    EXPECT_DOUBLE_EQ(bodySpecie->m, 10.0);
    EXPECT_DOUBLE_EQ(bodySpecie->FeretMax, 5.5);

    EXPECT_DOUBLE_EQ(bodySpecie->Ine_1[0][0], 1.0);
    EXPECT_DOUBLE_EQ(bodySpecie->Ine_1[1][1], 1.0);
    EXPECT_DOUBLE_EQ(bodySpecie->Ine_1[2][2], 1.0);
}

/**
 * @brief Test edge case of LoadFromFile with an empty file.
 */
TEST_F(BodySpecieTest, LoadFromFileEmpty) {
    FILE *ft = fopen("empty_body_specie.txt", "w+");
    fclose(ft);

    ft = fopen("empty_body_specie.txt", "r");
    ASSERT_NE(ft, nullptr);
    bodySpecie->LoadFromFile(ft);
    fclose(ft);

    EXPECT_EQ(bodySpecie->Ng, 0);
    EXPECT_TRUE(bodySpecie->num.empty());
    EXPECT_TRUE(bodySpecie->xl.empty());
    EXPECT_TRUE(bodySpecie->yl.empty());
    EXPECT_TRUE(bodySpecie->zl.empty());
    EXPECT_TRUE(bodySpecie->rl.empty());
    EXPECT_DOUBLE_EQ(bodySpecie->m, 0);
    EXPECT_NEAR(bodySpecie->FeretMax, 0.0, 1e-6);
}

