#pragma once

#include <iostream>
#include "Velocity.h"
#include "../ComputingForce.h"

class Contact;
class Configuration;
class Body;
class Sphere;

class Elbow {
public:
	Elbow() noexcept;
	void ReadFromFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft) const noexcept;
	void WriteOutFile(FILE *ft, int mode) const noexcept;
	void Move(double time, double t) noexcept;
	double Vmax() const noexcept;
	double Wmax() const noexcept;
	double Delay() const noexcept;

	int numero;
public:
	// Position of center of mass
	double x,y,z;
	// Base
	double nx,ny,nz;
	double tx,ty,tz;
	double sx,sy,sz;
	// Velocity
	double vx,vy,vz;
	double wx,wy,wz;
	Velocity V;
	// Linked cell
	int *Cell,NCell;
	int *Cell2,NCell2;
	
	double r,R,alpha,cx,cy,cz;
	double xi,yi,zi;
	double xf,yf,zf;
};
