//
// Created by ludfr on 25/02/25.
//

#include "Monitoring.h"

#include <iostream>

Monitoring::Monitoring(std::string id, std::string filename): id(std::move(id)),
                                                              filename(std::move(filename)) {
}

void Monitoring::metrics(const double current, const double final) const {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    char command[1024];
    sprintf(command, "/home/ludfr/.venv/bin/python3 /home/ludfr/monitoring/bridgeToInflux.py %s %f %f", id.c_str(), current, final);
    system(command);
}