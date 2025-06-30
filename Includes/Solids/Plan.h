#pragma once

#include "Solid.h"

class Contact;
class Configuration;
class Sphere;
class Body;

class Plan : public Solid{
public:
	Plan() noexcept;
	~Plan() noexcept;

	[[nodiscard]] double Ds() const noexcept { return ds; }
	[[nodiscard]] double Dt() const noexcept { return dt; }
    [[nodiscard]] int Periodic() const noexcept { return periodic; }
    [[nodiscard]] int ListCount() const noexcept { return Nlist; }
    [[nodiscard]] double Sigma() const noexcept { return sigma; }
    [[nodiscard]] int List(const int i) const { return list[i]; }
    [[nodiscard]] int InAndOut() const noexcept { return inAndOut; }

    void ListCount(const int rhs) noexcept {  this->Nlist = rhs; }
    void List(const int i, const int rhs) const {  this->list[i] = rhs; }

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
	double sigma;
};

