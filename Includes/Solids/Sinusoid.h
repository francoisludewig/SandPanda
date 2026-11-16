#pragma once

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

class Sinusoid{
public:
	Sinusoid() noexcept;
	Sinusoid(double a0, double a1, double w, double phi) noexcept;
	
	Sinusoid(const Sinusoid& other) noexcept = default;
	Sinusoid(Sinusoid&& other) noexcept = default;
	Sinusoid& operator=(const Sinusoid& other) noexcept = default;
	Sinusoid& operator=(Sinusoid&& other) noexcept = default;
	
	[[nodiscard]] double Value(const double t) const noexcept { return (a0+a1*sin(w*t+phi)); }

	void LoadFromFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft, int mode) const noexcept;
	void Display() const noexcept;
	[[nodiscard]] double Max() const noexcept;
	[[nodiscard]] double DelayTr() const noexcept;
	[[nodiscard]] double DelayRot() const noexcept;

private:
	double a0,a1,w,phi;
};
