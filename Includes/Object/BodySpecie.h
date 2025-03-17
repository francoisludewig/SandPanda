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

