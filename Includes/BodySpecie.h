#pragma once

#include <iostream>
#include <vector>

class BodySpecie{		
public:
	BodySpecie() noexcept;
	~BodySpecie() noexcept;
	
	void LoadFromFile(FILE *ft) noexcept;
	void FreeMemory() noexcept;
	void Display() const noexcept;
	
	int SphereCount() const noexcept { return Ng; }
	double SphereX(int i) const noexcept { return xl[i]; }
	double SphereY(int i) const noexcept { return yl[i]; }
	double SphereZ(int i) const noexcept { return zl[i]; }
	double SphereRadius(int i) const noexcept { return rl[i]; }
	double GetFeretMax() const noexcept { return FeretMax; }
	double Mass() const noexcept { return m; }
	double Intertie(int i, int j) const noexcept { return Ine_1[i][j]; }

private:
	int sp;
	int Ng;
	std::vector<int> num;
	std::vector<double> xl;
	std::vector<double> yl;
	std::vector<double> zl;
	std::vector<double> rl;
	double m,FeretMax;
	double Ine_1[3][3];
};

