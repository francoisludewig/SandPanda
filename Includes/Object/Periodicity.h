#pragma once

#include <vector>
#include <iostream>

class Sphere;
class Body;
class Plan;

void PeriodicityPL(const int & Nsph, const int & Nbd, const int &Npl, std::vector<Sphere> & sph, std::vector<Body> & bd, std::vector<Plan> & pl) noexcept;
