#pragma once

#include "Solid.h"

class Contact;
class Configuration;
class Body;
class Cone;

class PlanR : public Solid{
public:
	PlanR() noexcept;
	void readFromFile(FILE *ft) noexcept;
	void writeToFile(FILE *ft) const noexcept;
	void writeOutFile(FILE *ft, int mode) const noexcept;

	[[nodiscard]] double Radius() const noexcept { return r; }

private:
	double r,dn;
	int periodic;
};
