#pragma once

#include "Solid.h"

class Contact;
class Data;
class Sphere;
class Body;

class Plan : public Solid{
public:
	Plan() noexcept;
	~Plan() noexcept;
	void InitList(int N) noexcept;
	double Ds() const noexcept;
	double Dt() const noexcept;
	void LoadFromFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft) const noexcept;
	void WriteOutFile(FILE *ft, int mode) const noexcept;
	void Display() const noexcept;
	void SetAlpha(double a) noexcept;
	void Normal(Contact *c, Sphere *s) noexcept;
	
public:
	double dt,ds,dn;
	int periodic;
	int *list;
	int Nlist;
	int inAndOut;
	double sigma,ra;	
};

