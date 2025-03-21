// ContactIdentifier_test.cpp
// Unit tests for the ContactIdentifier class using Google Test.

#include <gtest/gtest.h>
#include "../../Includes/Contact/ContactIdentifier.h"

// Issues detected:
// 1. Bitwise OR (`||`) is used instead of bitwise OR (`|`) in computeIdentifier, causing logical errors.
// 2. Potential integer overflow if `other`, `other_sph`, or `sph` exceed 16 or 8-bit storage assumptions.
// 3. No documentation on expected behavior for negative values or large integers.


// Test cases for computeIdentifier with one parameter
TEST(ContactIdentifierTest, ComputeIdentifier_OneParam) {
    uint64_t id = ContactIdentifier::computeIdentifier(SPHERE_VS_SPHERE, 5);
    uint64_t expected = static_cast<uint8_t>(SPHERE_VS_SPHERE);
    expected = expected << 56 | 5 << 16;
    EXPECT_EQ(id, expected);
}

// Test cases for computeIdentifier with two parameters
TEST(ContactIdentifierTest, ComputeIdentifier_TwoParams) {
    uint64_t id = ContactIdentifier::computeIdentifier(SPHERE_VS_BODY, 10, 20);
    uint64_t expected = static_cast<uint8_t>(SPHERE_VS_BODY);
    expected = expected << 56 | 10 << 16 | 20 << 8;
    EXPECT_EQ(id, expected);
}

// Test cases for computeIdentifier with three parameters
TEST(ContactIdentifierTest, ComputeIdentifier_ThreeParams) {
    uint64_t id = ContactIdentifier::computeIdentifier(BODY_VS_PLAN, 15, 25, 35);
    uint64_t expected = static_cast<uint8_t>(BODY_VS_PLAN);
    expected = expected << 56 | 15 << 16 | 25 << 8 | 35;
    EXPECT_EQ(id, expected);
}

// Boundary tests
TEST(ContactIdentifierTest, ComputeIdentifier_BoundaryValues) {
    uint64_t id = ContactIdentifier::computeIdentifier(BODY_VS_ELBOW, 0, 0, 0);
    uint64_t expected = static_cast<uint8_t>(BODY_VS_ELBOW);
    expected = expected << 56;
    EXPECT_EQ(id, expected);
    id = ContactIdentifier::computeIdentifier(BODY_VS_ELBOW, 4294967295, 255, 255); // Maximum values
    uint64_t expected2 = static_cast<uint8_t>(BODY_VS_ELBOW);
    expected2 = ((expected2 << 56) | (4294967295 << 16) | (255 << 8) | 255);
    EXPECT_EQ(id, expected2);
}

// Edge cases for overflow
TEST(ContactIdentifierTest, ComputeIdentifier_OverflowValues) {
    uint64_t id = ContactIdentifier::computeIdentifier(SPHERE_VS_CONE, 42949672955, 300, 300);
    uint64_t expected = static_cast<uint8_t>(SPHERE_VS_CONE);
    expected = expected << 56 | (42949672955 & 0xFFFFFFFF) << 16 | (44 & 0xFF) << 8 | (44 & 0xFF);
    EXPECT_EQ(id, expected);
}
