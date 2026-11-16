#pragma once

#include <vector>

class Sphere;
class Configuration;

class LinkedCellFiller{
public:
	static void Fill(std::vector<Sphere> & sph, const Configuration& dat, std::vector<Sphere*>& cell) noexcept;
};
