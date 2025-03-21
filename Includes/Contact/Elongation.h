#pragma once
#include <iostream>
#include <limits>
#include <cmath>

class Elongation{
public:
	Elongation() noexcept;
	
	Elongation(const Elongation& other) noexcept = default;
	Elongation(Elongation&& other) noexcept = default;
	Elongation& operator=(const Elongation& other) noexcept = default;
	Elongation& operator=(Elongation&& other) noexcept = default;

	void Reset() noexcept;
	void Display() const noexcept;
	void Rotate(double nx, double ny, double nz) noexcept;

    bool operator==(const Elongation& other) const noexcept {
        constexpr double epsilon = std::numeric_limits<double>::epsilon();
        return (std::fabs(x - other.x) < epsilon &&
                std::fabs(y - other.y) < epsilon &&
                std::fabs(z - other.z) < epsilon &&
                status == other.status);
    }

public:
	double x,y,z;
	int status;
};
