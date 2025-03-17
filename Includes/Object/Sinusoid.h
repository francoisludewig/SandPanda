#pragma once

#include <iostream>

class Sinusoid{
public:
	Sinusoid() noexcept;
    explicit Sinusoid(double a0) noexcept;
	Sinusoid(double a0, double a1, double w, double phi) noexcept;
	
	Sinusoid(const Sinusoid& other) noexcept = default;
	Sinusoid(Sinusoid&& other) noexcept = default;
	Sinusoid& operator=(const Sinusoid& other) noexcept = default;
	Sinusoid& operator=(Sinusoid&& other) noexcept = default;
	
	void LoadFromFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft, int mode) const noexcept;
	void Display() const noexcept;
	[[nodiscard]] double Value(double t) const noexcept;
    [[nodiscard]] double Max() const noexcept;
	[[nodiscard]] double DelayTr() const noexcept;
    [[nodiscard]] double DelayRot() const noexcept;

private:
	double a0,a1,w,phi;
};
