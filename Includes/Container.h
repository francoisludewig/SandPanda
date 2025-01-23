/*
 *  Plan.h
 *  
 *
 *  Created by fanfan on 14/07/10.
 *  Copyright 2010 ULg. All rights reserved.
 *
 */
#ifndef solid_h
#define solid_h 

#include <AppliedVelocities.h>
#include "Gravity.h"
#include <vector>

using LuGa::SandPanda::AppliedVelocities;
using namespace std;

class Sphere;
class Contact;
class Fluid;

class Container{
//protected:
public:
	int numero;
	// Position of center of mass
	double x,y,z;
	// Base
    double q0,q1,q2,q3;
	double nx,ny,nz;
	double tx,ty,tz;
	double sx,sy,sz;
	// Velocity
	double vx,vy,vz;
	double wx,wy,wz;
	// Acceleration	
	double Fx,Fy,Fz;
	double Mx,My,Mz;
	double Fcx,Fcy,Fcz;
	double Mcx,Mcy,Mcz;
	double Mass,In,It,Is;
	int Force,activeGravity;
	AppliedVelocities V;
	// Linked cell
	int *Cell,NCell;
	int *Cell2,NCell2;
	// List of grain boder
	int *num;
	double *GBx,*GBy,*GBz;
	double R[3][3];	
	double xMemory,yMemory,zMemory;	
	int ControlGB;
    class Cell **vcell;
public:
	int Ngb;
	Container();
	~Container();
  class Cell** getVcell();
	void readFromFile(FILE *ft);
	void readAccelerationFromFile(FILE *ft);
	void writeAccelerationFromFile(FILE *ft);
	void writeToFile(FILE *ft);
	void writeOutFile(FILE *ft, int mode);
    void ComputeBase();
	void affiche();
	void upDateForce();
	void upDateVelocity(double time, double dt, Gravity gt);
	void upDateGravityVelocity(double time, double dt, Gravity gt);
	void move(double dt);
	void moveGravity(double dt, Gravity gt);
	void upDateLinkedSphere(vector<Sphere> & sph, double time, Gravity gt);
	void upDateGravityLinkedSphere(vector<Sphere> & sph, double time, Gravity gt);
	double Vmax();	
	double Wmax();
	double Delay();
	void initTimeStep();
	void OnOffGravity(bool OnOff);
	void SetVelocityToZero();
	void setMemoryPosition();
	void getMemoryPosition();
	void UpdateForceFromGB(int & Nsph,vector<Sphere> & sph);
	void UpdateGravityForceFromGB(int & Nsph,vector<Sphere> & sph, Gravity gt);
	void setVx(double newA0, double newA1, double newW, double newPhi);
	void setWx(double newA0, double newA1, double newW, double newPhi);
	void setVy(double newA0, double newA1, double newW, double newPhi);
	void setWy(double newA0, double newA1, double newW, double newPhi);
	void setVz(double newA0, double newA1, double newW, double newPhi);
	void setWz(double newA0, double newA1, double newW, double newPhi);
	double ValueOfVx(double t);
	double ValueOfVy(double t);
	double ValueOfVz(double t);	
	double ValueOfWx(double t);
	double ValueOfWy(double t);
	double ValueOfWz(double t);
	void upDateVelocityLinkedSphere(vector<Sphere> & sph, double time);
	void upDateGravityVelocityLinkedSphere(vector<Sphere> & sph, double time, Gravity gt);
	void setMass(double m);
	void setFcx(double fx);
	void setFcy(double fy);
	void setFcz(double fz);
	double getFcy();
	void setControlGB(int v,vector<Sphere> & sph);
};

#endif 
