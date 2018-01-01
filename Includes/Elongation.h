#pragma once
#include <iostream>

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
	
public:
	double x,y,z;
	int status;
};
