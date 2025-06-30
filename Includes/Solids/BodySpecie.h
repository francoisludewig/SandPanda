#pragma once

#include <iostream>
#include <vector>

class BodySpecie{		
public:
	BodySpecie() noexcept;
	~BodySpecie() noexcept;
	
	void LoadFromFile(FILE *ft) noexcept;
	void Display() const noexcept;
	
	[[nodiscard]] int SphereCount() const noexcept { return Ng; }
	[[nodiscard]] double SphereX(const int i) const noexcept { return xl[i]; }
	[[nodiscard]] double SphereY(const int i) const noexcept { return yl[i]; }
	[[nodiscard]] double SphereZ(const int i) const noexcept { return zl[i]; }
	[[nodiscard]] double SphereRadius(const int i) const noexcept { return rl[i]; }
	[[nodiscard]] double GetFeretMax() const noexcept { return FeretMax; }
	[[nodiscard]] double Mass() const noexcept { return m; }
	[[nodiscard]] double Intertie(const int i, const int j) const noexcept { return Ine_1[i][j]; }

private:
	int Ng;
	std::vector<int> num;
	std::vector<double> xl;
	std::vector<double> yl;
	std::vector<double> zl;
	std::vector<double> rl;
	double m,FeretMax;
	double Ine_1[3][3];
};

