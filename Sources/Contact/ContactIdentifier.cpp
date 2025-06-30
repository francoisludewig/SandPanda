//
// Created by ludfr on 19-03-25.
//

#include "../../Includes/Contact/ContactIdentifier.h"


uint64_t ContactIdentifier::computeIdentifier(CONTACT_TYPE contactType, uint32_t other) {
    return computeIdentifier(contactType, other, 0, 0);
}

uint64_t ContactIdentifier::computeIdentifier(CONTACT_TYPE contactType, uint32_t other, uint8_t other_sph) {
    return computeIdentifier(contactType, other, other_sph, 0);
}

uint64_t ContactIdentifier::computeIdentifier(CONTACT_TYPE contactType, uint32_t other, uint8_t other_sph, uint8_t sph) {
    uint64_t value = static_cast<uint8_t>(contactType);
    value = value << 56 | static_cast<uint64_t>(other) << 16 | static_cast<uint16_t>(other_sph) << 8 | sph;
    return value;
}
