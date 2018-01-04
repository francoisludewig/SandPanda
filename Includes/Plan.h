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

	double Ds() const noexcept { return ds; }
	double Dt() const noexcept { return dt; }
    int Periodic() const noexcept { return periodic; }
    int ListCount() const noexcept { return Nlist; }
    double Sigma() const noexcept { return sigma; }
    int List(int i) const { return list[i]; }
    int InAndOut() const noexcept { return inAndOut; }

    void ListCount(int rhs) noexcept {  this->Nlist = rhs; }
    void List(int i, int rhs) const {  this->list[i] = rhs; }

	void InitList(int N) noexcept;
	void LoadFromFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft) const noexcept;
	void WriteOutFile(FILE *ft, int mode) const noexcept;
	void Display() const noexcept;
	void SetAlpha(double a) noexcept;
	void Normal(Contact *c, Sphere *s) noexcept;
	
private:
	double dt,ds,dn;
	int periodic;
	int *list;
	int Nlist;
	int inAndOut;
	double sigma,ra;	
};

