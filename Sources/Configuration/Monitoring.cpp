//
// Created by ludfr on 25/02/25.
//

#include "../../Includes/Configuration/Monitoring.h"
#include <iostream>
#include <unistd.h>

void Monitoring::initialize(std::string& idName, std::string &path) {
    this->id = idName;
    this->scriptPath = path;
    pid = getpid();
}

void Monitoring::metrics(double current, double final) const {
    const auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    char command[1024];
    sprintf(command, "/home/ludfr/.venv/bin/python3 %s %s %f %f %d", scriptPath.c_str(), id.c_str(), current, final, pid);
    system(command);
}