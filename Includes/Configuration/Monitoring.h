//
// Created by ludfr on 25/02/25.
//

#pragma once

#include <string>
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

    void initialize(std::string& idName, std::string &path);
    void metrics(double current, double final) const;

private:
    Monitoring() noexcept = default;
    std::string id{};
    std::string scriptPath{};
    int pid = -1;
};
