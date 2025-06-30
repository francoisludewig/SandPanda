#pragma once

#include "Solid.h"
#include <vector>

class Contact;
class Configuration;
class Body;
class PlanR;

class Cone:public Solid{
public:
	Cone() noexcept;
	void readFromFile(FILE *ft) noexcept;
	void writeToFile(FILE *ft) const noexcept;
	void writeOutFile(FILE *ft, int mode) const noexcept;
	void LimitLink(std::vector<PlanR> & plr) noexcept;
	void LimitForce() noexcept;
	void LimitUpdate() noexcept;

	[[nodiscard]] double BottomRadius() const noexcept { return r0; }
	[[nodiscard]] double TopRadius() const noexcept { return r1; }
	[[nodiscard]] double Height() const noexcept { return h; }
	[[nodiscard]] int In() const noexcept { return in; }

private:
	double r0,r1,h,dr;
	int periodic,in;
    int numTop,numBottom;
    double lxTop,lyTop,lzTop;
    double lxBottom,lyBottom,lzBottom;
    PlanR *top,*bottom;

};
