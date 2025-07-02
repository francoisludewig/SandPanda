//
// Created by ludfr on 19-03-25.
//

#include "../../Includes/Elongations/ElongationManager.h"

ElongationManager::ElongationManager(const int maxContact) noexcept {
    xsi.reserve(maxContact);
    tp_xsi.reserve(maxContact);
    id.reserve(maxContact);
    tp_id.reserve(maxContact);

    for (int i = 0; i < maxContact; i++) {
        Elongation elongation;
        xsi.push_back(elongation);
        Elongation tp_elongation;
        tp_xsi.push_back(tp_elongation);
        id.push_back(0);
        tp_id.push_back(0);
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

void ElongationManager::writeToFile(FILE *ft) const noexcept {
    fprintf(ft,"%d\n",tp_count);
    for(int i = 0 ; i < tp_count ; i++)
        fprintf(ft,"%lu\t%d\t%.16e\t%.16e\t%.16e\n",tp_id[i], tp_xsi[i].status,tp_xsi[i].x,tp_xsi[i].y,tp_xsi[i].z);
}

void ElongationManager::readFromFile(FILE *ft) noexcept {
    fscanf(ft, "%d\n", &tp_count);
    for(int i = 0 ; i < tp_count ; i++)
        fscanf(ft,"%lu\t%d\t%lf\t%lf\t%lf\n",&tp_id[i],&tp_xsi[i].status,&tp_xsi[i].x,&tp_xsi[i].y,&tp_xsi[i].z);
}