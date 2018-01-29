#include "../Includes/Compaction.h"

#include "../Includes/Solids/Velocity.h"
#include "../Includes/Gravity.h"
#include "../Includes/Solids/Plan.h"
#include "../Includes/Solids/PlanR.h"
#include "../Includes/Solids/Cone.h"
#include "../Includes/Solids/Elbow.h"
#include "../Includes/Solids/Sphere.h"
#include "../Includes/Solids/Body.h"
#include "../Includes/Contact/Contact.h"
#include "../Includes/ReadWrite.h"
#include "../Includes/Contact/ContactDetection.h"
#include "../Includes/ComputingForce.h"
#include "../Includes/Move.h"
#include "../Includes/Evolution.h"
#include "../Includes/Data.h"
#include "../Includes/Solids/HollowBall.h"
#include "../Includes/LinkedCells/CellBounds.h"

void Compaction::Secousse(vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co, double Gamma, double f, Data & dat) noexcept {
	double w = 2*M_PI*f;
	double A = Gamma*9.81/w/w;
	
	printf("A = %e, w = %e , Aw = %e\n",A,w,A*w);
	
	for(auto& plan : pl)
		plan.SetVz(0,A*w,-w,M_PI);
	
	for(auto& disk : plr)
		disk.SetVz(0,A*w,-w,M_PI);
	
	for(auto& cone : co)
		cone.SetVz(0,A*w,-w,M_PI);
	
	dat.Total += 1./f;
	int Ntsp = (1./f)/(dat.dt) + 1;
	dat.dt = (1./f)/(double)(Ntsp);
	printf("dt = %e\n",dat.dt);
	
}

void Compaction::Relaxation(vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co, Data & dat) noexcept {
	for(auto& plan : pl)
			plan.SetVz(0,0,0,0);

		for(auto& disk : plr)
			disk.SetVz(0,0,0,0);

		for(auto& cone : co)
			cone.SetVz(0,0,0,0);
	
	dat.Total += 0.5;
}


int Compaction::Run(vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,vector<Elbow> & elb,vector<Sphere> & sph,vector<Body> & bd,vector<HollowBall> & hb,Data & dat,Gravity & gf,
		std::vector<Sphere*>& cell, int & Ntp, char *name,int ntpi, int ntpf, double Gamma, double f, int Nthreshold, const CellBounds& cellBounds) noexcept {

	Evolution evolution(sph.size(), bd.size(), pl ,plr, co, elb, dat, gf, cellBounds);
	dat.record = 0;
	for(int nt = ntpi  ; nt <= ntpf ; nt++){
		//Secousse
		Secousse(pl,plr,co,Gamma,f,dat);
		printf("Total = %e\n",dat.Total);
		Ntp = evolution.Evolve(pl,plr,co,elb,sph,bd,hb,dat,gf,cell,Ntp, name,Nthreshold);
		// Relaxation
		Relaxation(pl,plr,co,dat);
		printf("Total = %e\n",dat.Total);
		Ntp = evolution.Evolve(pl,plr,co,elb,sph,bd,hb,dat,gf,cell,Ntp, name,Nthreshold);
		
		if(dat.record == 0){
			// Enregistrement
			ReadWrite::writeStartStopContainer(name,pl,plr,co,elb);
			ReadWrite::writeStartStopSphere(name,sph);
			ReadWrite::writeStartStopBodies(name,bd,sph);
			ReadWrite::writeStartStopData(name, &gf, &dat);
			ReadWrite::writeStartStopHollowBall(name, hb);
			
			ReadWrite::writeOutContainer(name,nt,pl,plr,co,elb,dat.outMode);
			ReadWrite::writeOutSphere(name,nt,sph,dat.outMode);
			ReadWrite::writeOutBodies(name,nt,bd,dat.outMode);
			//writeOutData(name, nt, &gf, &dat);
			ReadWrite::writeOutHollowBall(name, Ntp, hb);
			printf("Ntp = %d\n",nt);
			Ntp++;
		}
	}
	return Ntp;
}
