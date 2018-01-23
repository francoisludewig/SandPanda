#pragma once

#include <vector>

class Sphere;
class Data;

class LinkedCellFiller{
public:
	static void Fill(std::vector<Sphere> & sph, const Data& dat, std::vector<Sphere*>& cell) noexcept;
};
