//
// Created by ludfr on 19-03-25.
//

#include "../../Includes/Contact/ElongationManager.h"

ElongationManager::ElongationManager(int maxContact) noexcept {
    xsi.reserve(maxContact);
    tp_xsi.reserve(maxContact);
    id.reserve(maxContact);
    tp_id.reserve(maxContact);

    std::fill(id.begin(), id.end(), 0);
    std::fill(tp_id.begin(), tp_id.end(), 0);

    for (int i = 0; i < maxContact; i++) {
        Elongation elongation;
        xsi.push_back(elongation);
        Elongation tp_elongation;
        tp_xsi.push_back(tp_elongation);
    }
    count = 0;
    tp_count = 0;
}

void ElongationManager::InitXsi() noexcept {
    std::swap(xsi, tp_xsi);
    std::swap(id, tp_id);
    count = tp_count;
    tp_count = 0;
}

bool ElongationManager::AddXsi(Elongation &e, uint64_t contact_id) noexcept {
    // TODO Overflow management!!!
    if(tp_count == tp_xsi.size()) {
        return false;
    }
    tp_xsi[tp_count] = e;
    tp_id[tp_count] = contact_id;
    tp_count++;
    return true;
}

Elongation ElongationManager::FoundIt(uint64_t contact_id) const noexcept {
    for (int i = 0; i < count; i++) {
        if (id[i] == contact_id) {
            return xsi[i];
        }
    }
    return {};
}