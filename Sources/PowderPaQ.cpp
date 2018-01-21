#include "../Includes/PowderPaQ.h"

#include "../Includes/Velocity.h"
#include "../Includes/Gravity.h"
#include "../Includes/Plan.h"
#include "../Includes/PlanR.h"
#include "../Includes/Cone.h"
#include "../Includes/Elbow.h"
#include "../Includes/Sphere.h"
#include "../Includes/Body.h"
#include "../Includes/Contact.h"
#include "../Includes/ReadWrite.h"
#include "../Includes/ContactDetection.h"
#include "../Includes/ComputingForce.h"
#include "../Includes/Move.h"
#include "../Includes/Evolution.h"
#include "../Includes/Data.h"

void PowderPaQ::PowderPaQsecousseUpward(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Data & dat, double PQheight, double PQVel) noexcept {
	for(auto& plan : pl){
		plan.SetVz(PQVel,0,0,0);
		plan.SetMemoryPosition();
	}
	for(auto& disk : plr){
		disk.SetVz(PQVel,0,0,0);
		disk.SetMemoryPosition();
	}
	for(auto& cone : co){
		cone.SetVz(PQVel,0,0,0);
		cone.SetMemoryPosition();
	}		
	dat.Total += PQheight/PQVel;
}

void PowderPaQ::PowderPaQsecousseDownward(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Data & dat, double PQheight) noexcept {
	double delay = sqrt(2*PQheight/9.81);
	dat.Total += delay;	
	int Ntsp = (delay)/(dat.dt) + 1;
	dat.dt = (delay)/(double)(Ntsp);
	//printf("dt = %e\n",dat.dt);
	for(auto& plan : pl){
		plan.SetVz(0,0,0,0);
		plan.OnOffGravity(true);
		plan.SetVelocityToZero();
	}
	for(auto& disk : plr){
		disk.SetVz(0,0,0,0);
		disk.OnOffGravity(true);
		disk.SetVelocityToZero();
	}
	for(auto& cone : co){
		cone.SetVz(0,0,0,0);
		cone.OnOffGravity(true);
		cone.SetVelocityToZero();
	}	
}

void PowderPaQ::PowderPaQrelaxation(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Data & dat, double t, double PQheight, double PQVel) noexcept {
	for(auto& plan : pl){
		plan.SetVz(0,0,0,0);
		plan.OnOffGravity(false);
		plan.GetMemoryPosition();
	}
	for(auto& disk : plr){
		disk.SetVz(0,0,0,0);
		disk.OnOffGravity(false);
		disk.GetMemoryPosition();
	}
	for(auto& cone : co){
		cone.SetVz(0,0,0,0);
		cone.OnOffGravity(false);
		cone.GetMemoryPosition();
	}
	dat.Total += (t-PQheight/PQVel-sqrt(2*PQheight/9.81));	
}

int PowderPaQ::PowderPaQRun(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Data & dat,Gravity & gf,
		Sphere *cell[], int & Ntp, char *name,bool record,int ntpi, int ntpf, int Nthreshold, double PQheight, double PQVel) noexcept {
	Evolution evolution(sph.size(), bd.size());
	record = 0;
	for(int nt = ntpi  ; nt <= ntpf ; nt++){
		//Secousse
		PowderPaQsecousseUpward(pl,plr,co,dat,PQheight,PQVel);
		Ntp = evolution.Evolve(pl,plr,co,elb,sph,bd,hb,dat,gf,cell,Ntp, name,record,Nthreshold);

		PowderPaQsecousseDownward(pl,plr,co,dat,PQheight);
		Ntp = evolution.Evolve(pl,plr,co,elb,sph,bd,hb,dat,gf,cell,Ntp, name,record,Nthreshold);

		// Relaxation
		if(nt < 100)
			PowderPaQrelaxation(pl,plr,co,dat,0.5,PQheight,PQVel);
		else
			PowderPaQrelaxation(pl,plr,co,dat,0.25,PQheight,PQVel);

		Ntp = evolution.Evolve(pl,plr,co,elb,sph,bd,hb,dat,gf,cell,Ntp, name,record,Nthreshold);
		// Enregistrement
		ReadWrite::writeStartStopContainer(name,pl,plr,co,elb);
		ReadWrite::writeStartStopSphere(name,sph);
		ReadWrite::writeStartStopBodies(name,bd,sph);
		ReadWrite::writeStartStopData(name, &gf, &dat);
		ReadWrite::writeStartStopHollowBall(name, hb);

		ReadWrite::writeOutContainer(name,nt,pl,plr,co,elb,dat.outMode);
		ReadWrite::writeOutSphere(name,nt,sph,dat.outMode);
		ReadWrite::writeOutBodies(name,nt,bd,dat.outMode);
		ReadWrite::writeOutData(name, nt, &gf, &dat);
		ReadWrite::writeOutContact(name,nt,evolution.ContactCount(),evolution.GetContacts(),dat);
		ReadWrite::writeOutHollowBall(name, Ntp, hb);
		Ntp++;

	}
	return Ntp;
}
