#pragma once

#include "Solid.h"

class Contact;
class Data;
class Body;
class Cone;

class PlanR : public Solid{
public:
	double r,dn;
	int periodic;
	double alpha;
public:
	PlanR() noexcept;
	void readFromFile(FILE *ft) noexcept;
	void writeToFile(FILE *ft) const noexcept;
	void writeOutFile(FILE *ft, int mode) const noexcept;
};
