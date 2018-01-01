#pragma once

#include <iostream>
#include <vector>

class Plan;
class PlanR;
class Cone;
class Elbow;
class HollowBall;
class Sphere;
class Data;
class Body;
class Gravity;
class Contact;
class BodySpecie;

class ReadWrite {
public:
	static void readOutContainer(char *directory, int & Npl, int & Nplr, int & Nco, int & Nelb, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb) noexcept;
	static void writeStartStopContainer(char *directory, int & Npl, int & Nplr, int & Nco, int & Nelb, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb) noexcept;
	static void writeOutContainer(char *directory, int n,int & Npl, int & Nplr, int & Nco, int & Nelb, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, int mode) noexcept;
	static void readStart_stopContainer(char *directory, int & Npl, int & Nplr, int & Nco, int & Nelb, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb) noexcept;
	
	static void readOutHollowBall(char *directory, int & Nhbl, std::vector<HollowBall> & hbl) noexcept;
	static void writeStartStopHollowBall(char *directory, int & Nhbl, std::vector<HollowBall> & hbl) noexcept;
	static void writeOutHollowBall(char *directory, int n, int & Nhbl, std::vector<HollowBall> & hbl) noexcept;
	static void readStart_stopHollowBall(char *directory, int & Nhbl, std::vector<HollowBall> & hbl) noexcept;
	
	static void readOutSphere(char *directory, int & Nsph, std::vector<Sphere> & sph, int limite) noexcept;
	static void writeStartStopSphere(char *directory, int & Nsph, std::vector<Sphere> & sph) noexcept;
	static void writeOutSphere(char *directory, int n, int & Nsph, std::vector<Sphere> & sph, int mode) noexcept;
	static void readStart_stopSphere(char *directory, int & Nsph, std::vector<Sphere> & sph, int limite) noexcept;
	
	static void readOutBodies(char *directory, int & Nbd, std::vector<Body> & bd, int limite) noexcept;
	static void writeStartStopBodies(char *directory, int & Nbd, std::vector<Body> & bd,std::vector<Sphere> & sph) noexcept;
	static void writeOutBodies(char *directory, int n, int & Nbd, std::vector<Body> & bd, int mode) noexcept;
	static void readStart_stopBodies(char *directory, int & Nbd, std::vector<Body> & bd, int limite) noexcept;
	
	static void readOutData(char *directory, Gravity *gf, Data *dat) noexcept;
	static void writeOutData(char *directory, int n, Gravity *gf, Data *dat) noexcept;
	static void writeStartStopData(char *directory, Gravity *gf, Data *dat) noexcept;
	static void readStartStopData(char *directory, Gravity *gf, Data *dat) noexcept;
	
	//void readOutType(char *directory, Data & dat);
	static void writeOutContact(char *directory, int n, int & Nct, Contact *ct, Data & dat) noexcept;
	static void writeOutContactDetails(char *directory, int n, int & Nct, Contact *ct, Data & dat) noexcept;
	static void readOutBodySpecie(char *directory, int & Nbdsp,std::vector<BodySpecie> & bdsp) noexcept;
	//void readExportLink(char *directory, std::vector<Link> &lk, double k);
};
