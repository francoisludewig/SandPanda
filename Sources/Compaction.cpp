#include "../Includes/Compaction.h"

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
#include "../Includes/HollowBall.h"

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


int Compaction::Run(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,
										vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,vector<Elbow> & elb,vector<Sphere> & sph,vector<Body> & bd,vector<HollowBall> & hb,Contact *ct,Data & dat,Gravity & gf,
										Sphere *cell[], int & Ntp, char *name,bool record,int ntpi, int ntpf, double Gamma, double f, int Nthreshold) noexcept {
	record = 0;
	for(int nt = ntpi  ; nt <= ntpf ; nt++){
		//Secousse
		Secousse(pl,plr,co,Gamma,f,dat);
		printf("Total = %e\n",dat.Total);
		Ntp = Evolution::Evolve(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,pl,plr,co,elb,sph,bd,hb,ct,dat,gf,cell,Ntp, name,record,Nthreshold);
		// Relaxation
		Relaxation(pl,plr,co,dat);
		printf("Total = %e\n",dat.Total);
		Ntp = Evolution::Evolve(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,pl,plr,co,elb,sph,bd,hb,ct,dat,gf,cell,Ntp, name,record,Nthreshold);
		
		if(record == 0){
			// Enregistrement
			ReadWrite::writeStartStopContainer(name,Npl,Nplr,Nco,Nelb,pl,plr,co,elb);
			ReadWrite::writeStartStopSphere(name,Nsph,sph);
			ReadWrite::writeStartStopBodies(name,Nbd,bd,sph);
			ReadWrite::writeStartStopData(name, &gf, &dat);
			ReadWrite::writeStartStopHollowBall(name, Nhb, hb);
			
			ReadWrite::writeOutContainer(name,nt,Npl,Nplr,Nco,Nelb,pl,plr,co,elb,dat.outMode);
			ReadWrite::writeOutSphere(name,nt,Nsph0,sph,dat.outMode);
			ReadWrite::writeOutBodies(name,nt,Nbd,bd,dat.outMode);
			//writeOutData(name, nt, &gf, &dat);
			ReadWrite::writeOutHollowBall(name, Ntp, Nhb, hb);
			printf("Ntp = %d\n",nt);
			Ntp++;
		}
	}
	return Ntp;
}
