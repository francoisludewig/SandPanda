//
// Created by ludfr on 19-03-25.
//
#include <gtest/gtest.h>
#include "../../Includes/Contact/ElongationManager.h"

// Classe de test pour ElongationManager
class ElongationManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialisation avant chaque test
        manager = new ElongationManager(3);  // Test avec 3 contacts
    }

    void TearDown() override {
        // Nettoyage après chaque test
        delete manager;
    }

    ElongationManager* manager{};
};

// Test du constructeur
TEST_F(ElongationManagerTest, Constructor_InitializesCorrectly) {
    EXPECT_NO_THROW(ElongationManager(5));
}

// Test de InitXsi
TEST_F(ElongationManagerTest, InitXsi_SwapsBuffers) {
    Elongation e;
    manager->AddXsi(e, 42);
    manager->InitXsi();

    // Après InitXsi, FoundIt(42) doit retourner l'élongation ajoutée
    auto found = manager->FoundIt(42);
    EXPECT_EQ(found, e);
}

// Test de AddXsi
TEST_F(ElongationManagerTest, AddXsi_StoresValuesCorrectly) {
    Elongation e1, e2;
    manager->AddXsi(e1, 101);
    manager->AddXsi(e2, 202);

    manager->InitXsi();

    EXPECT_EQ(manager->FoundIt(101), e1);
    EXPECT_EQ(manager->FoundIt(202), e2);
}

// Test de FoundIt - Contact existant
TEST_F(ElongationManagerTest, FoundIt_ReturnsCorrectElongation) {
    Elongation e;
    manager->AddXsi(e, 555);
    manager->InitXsi();

    EXPECT_EQ(manager->FoundIt(555), e);
}

// Test de FoundIt - Contact inexistant
TEST_F(ElongationManagerTest, FoundIt_ReturnsDefaultForUnknownId) {
    Elongation default_e;
    EXPECT_EQ(manager->FoundIt(999), default_e);
}


// Test de FoundIt - Contact inexistant
TEST_F(ElongationManagerTest, AddXsi_ReturnsFalseWhenFull) {
    Elongation e1, e2, e3, e4;
    EXPECT_TRUE(manager->AddXsi(e1, 101));
    EXPECT_TRUE(manager->AddXsi(e2, 101));
    EXPECT_TRUE(manager->AddXsi(e3, 101));
    EXPECT_FALSE(manager->AddXsi(e4, 101));
}
