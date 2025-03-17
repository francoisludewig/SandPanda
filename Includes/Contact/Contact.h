#pragma once

#include "Elongation.h"

class Sphere;
class Body;
class Plan;
class PlanR;
class Cone;
class Elbow;

class Contact{
public:
	Contact() noexcept;
	~Contact() noexcept;
	
	void TimeStepInitialization() noexcept;
	void WriteOutFile(FILE *ft, double time) const noexcept;
	void WriteOutFileDetails(FILE *ft, double time) const noexcept;
	void Display() const noexcept;
	void inFile(FILE *ft) noexcept;
	
	int type;
	int Stick;
	double px,py,pz;
	double delta;
	double nx,ny,nz;
	double tx,ty,tz;
	double xi,yi,zi,xf,yf,zf;
	double Fx,Fy,Fz;
	
	Sphere *sa,*sb;
	Body *ba,*bb;
	int nba,nbb;

    Plan *pa;
	PlanR *par;
	Cone *cn;
	Elbow *ew;
	Elongation xsil;
};
