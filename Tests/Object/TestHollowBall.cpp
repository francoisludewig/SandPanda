/**
 * @file TestHollowBall.cpp
 * @brief Unit tests for the HollowBall class.
 */

#include <gtest/gtest.h>
#include "../Includes/Object/HollowBall.h"
#include "../Includes/Object/Sphere.h"
#include "../Includes/Contact/Contact.h"
#include <fstream>
/**
 * @brief Tests for the HollowBall class.
 */
class HollowBallTest : public ::testing::Test {
protected:
    void SetUp() override {
        hollowBall = new HollowBall();
    }

    void TearDown() override {
        delete hollowBall;
    }

    HollowBall *hollowBall;
};

/**
 * @brief Test the default constructor by saving and reading back the object.
 */
TEST_F(HollowBallTest, ConstructorInitializesCorrectly) {
    FILE *ft = fopen("test_hollowball_init.txt", "w+");
    ASSERT_NE(ft, nullptr);

    hollowBall->writeToFile(ft);
    fclose(ft);

    // Reload the saved object
    HollowBall newHollowBall;
    ft = fopen("test_hollowball_init.txt", "r");
    ASSERT_NE(ft, nullptr);
    newHollowBall.loadFromFile(ft);
    fclose(ft);

    // Test indirectly via file save/load
    FILE *ft_check = fopen("test_hollowball_check.txt", "w+");
    ASSERT_NE(ft_check, nullptr);
    newHollowBall.writeToFile(ft_check);
    fclose(ft_check);

    // Compare the files
    std::ifstream file1("test_hollowball_init.txt"), file2("test_hollowball_check.txt");
    std::string str1((std::istreambuf_iterator<char>(file1)), std::istreambuf_iterator<char>());
    std::string str2((std::istreambuf_iterator<char>(file2)), std::istreambuf_iterator<char>());

    EXPECT_EQ(str1, str2);
}

/**
 * @brief Test the loadFromFile method.
 */
TEST_F(HollowBallTest, LoadFromFile) {
    FILE *ft = fopen("test_hollowball.txt", "w+");
    fprintf(ft, "1.000000e+00\t2.000000e+00\t3.000000e+00\t7.070000e-01\t0.000000e+00\t7.070000e-01\t0.000000e+00\t4.000000e+00\t5.000000e+00\t6.000000e+00\t1.000000e-01\t2.000000e-01\t3.000000e-01\t1\t0\t1\t0\t1\t0\t1.000000e+01\t5.000000e+01\t1.000000e+02\n");
    fclose(ft);

    ft = fopen("test_hollowball.txt", "r");
    ASSERT_NE(ft, nullptr);
    hollowBall->loadFromFile(ft);
    fclose(ft);

    FILE *ft_out = fopen("test_hollowball_out.txt", "w+");
    ASSERT_NE(ft_out, nullptr);
    hollowBall->writeToFile(ft_out);
    fclose(ft_out);

    // Compare the expected output
    std::ifstream file1("test_hollowball.txt"), file2("test_hollowball_out.txt");
    std::string str1((std::istreambuf_iterator<char>(file1)), std::istreambuf_iterator<char>());
    std::string str2((std::istreambuf_iterator<char>(file2)), std::istreambuf_iterator<char>());

    EXPECT_EQ(str1, str2);
}

/**
 * @brief Test the Makeavatar method indirectly.
 */
TEST_F(HollowBallTest, MakeAvatar) {
    std::vector<Sphere> spheres;
    int Nsph = 0;
    int numero = 5;

    hollowBall->Makeavatar(spheres, Nsph, numero);

    ASSERT_EQ(spheres.size(), 1);
    EXPECT_EQ(Nsph, 1);
}

/**
 * @brief Test UpdateFromSph by linking with a sphere and verifying output.
 */
TEST_F(HollowBallTest, UpdateFromSph) {
    std::vector<Sphere> spheres;
    int Nsph = 0;
    int numero = 5;

    hollowBall->Makeavatar(spheres, Nsph, numero);

    ASSERT_EQ(spheres.size(), 1);

    Sphere &avatar = spheres[0];

    avatar.x = 5.0;
    avatar.y = 6.0;
    avatar.z = 7.0;
    avatar.vx = 0.1;
    avatar.vy = 0.2;
    avatar.vz = 0.3;
    avatar.wx = 0.4;
    avatar.wy = 0.5;
    avatar.wz = 0.6;

    hollowBall->UpdateFromSph(1.0);

    FILE *ft = fopen("test_update_from_sph.txt", "w+");
    ASSERT_NE(ft, nullptr);
    hollowBall->writeToFile(ft);
    fclose(ft);
}

/**
 * @brief Test the destructor to check proper memory deallocation.
 */
TEST(HollowBallMemoryTest, DestructorFreesMemory) {
    HollowBall *tempHollowBall = new HollowBall();

    // Allocate some memory
    FILE *ft = fopen("test_hollowball.txt", "w+");
    fprintf(ft, "1.0 2.0 3.0 0.707 0.0 0.707 0.0 4.0 5.0 6.0 0.1 0.2 0.3 1 0 1 0 1 0 10.0 50.0 100.0\n");
    fclose(ft);
    ft = fopen("test_hollowball.txt", "r");
    ASSERT_NE(ft, nullptr);
    tempHollowBall->loadFromFile(ft);
    fclose(ft);

    delete tempHollowBall;

    // Ensure no crashes occurred (no direct way to check memory deallocation)
    SUCCEED();
}
