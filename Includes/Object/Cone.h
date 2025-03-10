#pragma once

#include "Solid.h"
#include <vector>

class Contact;
class Data;
class Body;
class PlanR;

class Cone:public Solid{
public:
	double r0,r1,h,dr;
	int periodic,in;
    int numTop,numBottom;
    double lxTop,lyTop,lzTop;
    double lxBottom,lyBottom,lzBottom;
    PlanR *top,*bottom;
public:
	Cone() noexcept;
	void readFromFile(FILE *ft) noexcept;
	void writeToFile(FILE *ft) const noexcept;
	void writeOutFile(FILE *ft, int mode) const noexcept;
	void LimitLink(std::vector<PlanR> & plr) noexcept;
	void LimitForce() noexcept;
	void LimitUpdate() noexcept;
};
