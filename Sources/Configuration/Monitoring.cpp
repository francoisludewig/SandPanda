//
// Created by ludfr on 25/02/25.
//

#include "../../Includes/Configuration/Monitoring.h"
#include <iostream>
#include <unistd.h>

void Monitoring::initialize(std::string& idName) {
    this->id = idName;
    pid = getpid();
}

void Monitoring::metrics(const double current, const double final) const {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    char command[1024];
    sprintf(command, "/home/ludfr/.venv/bin/python3 /home/ludfr/monitoring/bridgeToInflux.py %s %f %f %d", Monitoring::id.c_str(), current, final, pid);
    system(command);
}