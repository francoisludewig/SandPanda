//
// Created by ludfr on 25/02/25.
//

#pragma once

#include <string>
#include <utility>
#include <fstream>
#include <chrono>

class Monitoring {
public:
    explicit Monitoring(std::string id, std::string filename);
    void metrics(const double current, const double final) const;

private:
    std::string id;
    std::string filename;
    int pid;
};
