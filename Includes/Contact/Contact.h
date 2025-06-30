#pragma once

#include <cstdint>

#include "../Elongations/Elongation.h"

class Sphere;
class Body;
class Plan;
class PlanR;
class Cone;
class Elbow;

class Contact{

public:

	enum class Type{
	 SphereSphere = 0,
	 SphereBody = 10,
	 BodyBody = 20,
	 SpherePlan = 1,
	 SpherePlanR = 2,
	 SphereCone = 3,
	 SphereElbow = 4,
	 SphereHollowBall = 5,
	 BodyPlan = 11,
	 BodyPlanR = 12,
	 BodyCone = 13,
	 BodyElbow = 14,
	 BodyHollowBall = 6,
	 None = 100
	};

	Contact() noexcept;
	~Contact() noexcept;
	
	void TimeStepInitialization() noexcept;
	void WriteOutFile(FILE *ft, double time) const noexcept;
	void WriteOutFileDetails(FILE *ft, double time) const noexcept;
	void Display() const noexcept;
	void inFile(FILE *ft) noexcept;
	
	Type type;
	int Stick;
	double px,py,pz;
	double delta;
	double nx,ny,nz;
	double tx,ty,tz;
	double xi,yi,zi,xf,yf,zf;
	double Fx,Fy,Fz;
	double Fax,Fay,Faz;
	double Fbx,Fby,Fbz;
	double Max, May, Maz;
	double Mbx, Mby, Mbz;
	Sphere *sa,*sb;
	Body *ba,*bb;
	int nba,nbb;
	
	Plan *pa;
	PlanR *par;
	Cone *cn;
	Elbow *ew;
	Elongation xsi;
	uint64_t id_a_xsi, id_b_xsi;
};
