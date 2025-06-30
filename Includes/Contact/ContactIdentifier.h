//
// Created by ludfr on 19-03-25.
//

#pragma once

#include <cstdint>

enum CONTACT_TYPE {
    SPHERE_VS_SPHERE,
    SPHERE_VS_PLAN,
    SPHERE_VS_PLANR,
    SPHERE_VS_CONE,
    SPHERE_VS_ELBOW,
    SPHERE_VS_BODY,
    BODY_VS_BODY,
    BODY_VS_PLAN,
    BODY_VS_PLANR,
    BODY_VS_CONE,
    BODY_VS_ELBOW,
    SPHERE_VS_HOLLOWBALL,
    BODY_VS_HOLLOWBALL
};

class ContactIdentifier {
public:
    static uint64_t computeIdentifier(CONTACT_TYPE contactType, uint32_t other);
    static uint64_t computeIdentifier(CONTACT_TYPE contactType, uint32_t other, uint8_t other_sph);
    static uint64_t computeIdentifier(CONTACT_TYPE contactType, uint32_t other, uint8_t other_sph,uint8_t sph);
};