#pragma once

#include "Sinusoid.h"

class Gravity{
public:
	Gravity() noexcept;
	void LoadFromFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft) const noexcept;
	void Move(double time,double dt) noexcept;
	
	// Value of the gravity acceleration
	double G;
	double ngx,ngy,ngz;
	double ngx0,ngy0,ngz0;
	
private:
	double q0,q1,q2,q3;
	Sinusoid wx,wy,wz;
};

