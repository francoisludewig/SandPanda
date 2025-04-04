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
    Monitoring(const Monitoring& other) noexcept = default;
    Monitoring(Monitoring&& other) noexcept = default;
    Monitoring& operator=(const Monitoring& other) noexcept = default;
    Monitoring& operator=(Monitoring&& other) noexcept = default;

    static Monitoring& getInstance() {
        static Monitoring monitoring{};
        return monitoring;
    }

    void initialize(std::string& idName);
    void metrics(const double current, const double final) const;

private:
    Monitoring() noexcept = default;
    std::string id;
    int pid;
};
