#include "../../Includes/Dynamic/Compaction.h"

#include "../../Includes/Object/Velocity.h"
#include "../../Includes/Configuration/Gravity.h"
#include "../../Includes/Object/Plan.h"
#include "../../Includes/Object/PlanR.h"
#include "../../Includes/Object/Cone.h"
#include "../../Includes/Object/Elbow.h"
#include "../../Includes/Object/Sphere.h"
#include "../../Includes/Object/Body.h"
#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Repository/ReadWrite.h"
#include "../../Includes/Contact/ContactDetection.h"
#include "../../Includes/Contact/ComputingForce.h"
#include "../../Includes/Dynamic/Move.h"
#include "../../Includes/Dynamic/Evolution.h"
#include "../../Includes/Configuration/Data.h"
#include "../../Includes/Object/HollowBall.h"

void Compaction::Secousse(vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,int Npl, int Nplr, int Nco, double Gamma, double f, Data & dat) noexcept {
	int i;
	double w = 2*M_PI*f;
	double A = Gamma*9.81/w/w;
	
	printf("Vibration\nA = %e, w = %e , Aw = %e\n",A,w,A*w);
	
	for(i = 0 ; i < Npl ; i++)
		pl[i].SetVz(0,A*w,-w,M_PI);
	
	for(i = 0 ; i < Nplr ; i++)
		plr[i].SetVz(0,A*w,-w,M_PI);
	
	for(i = 0 ; i < Nco ; i++)
		co[i].SetVz(0,A*w,-w,M_PI);
	
	
	dat.Total += 1./f;
	int Ntsp = (1./f)/(dat.dt) + 1;
	dat.dt = (1./f)/(double)(Ntsp);
	printf("dt = %e\n",dat.dt);
}

void Compaction::Relaxation(vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,int Npl, int Nplr, int Nco, Data & dat) noexcept {
	int i;
	for(i = 0 ; i < Npl ; i++)
		pl[i].SetVz(0,0,0,0);
	
	for(i = 0 ; i < Nplr ; i++)
		plr[i].SetVz(0,0,0,0);
	
	for(i = 0 ; i < Nco ; i++)
		co[i].SetVz(0,0,0,0);
	
	dat.Total += 0.5;
}


int Compaction::Run(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,
										vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,vector<Elbow> & elb,vector<Sphere> & sph,vector<Body> & bd,vector<HollowBall> & hb,Contact *ct,Data & dat,Gravity & gf,
										Sphere *cell[], int & Ntp, char *name,bool record,int ntpi, int ntpf, double Gamma, double f, int Nthreshold) noexcept {
	record = 0;
	for(int nt = ntpi  ; nt <= ntpf ; nt++){
		//Secousse
		Secousse(pl,plr,co,Npl,Nplr,Nco,Gamma,f,dat);
		printf("Total = %e\n",dat.Total);
		Ntp = Evolution::Evolve(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,pl,plr,co,elb,sph,bd,hb,ct,dat,gf,cell,Ntp, name,record,Nthreshold);
		// Relaxation
		Relaxation(pl,plr,co,Npl,Nplr,Nco,dat);
		printf("Relaxation\nTotal = %e\n",dat.Total);
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


int Compaction::RunOMP(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,int & Ncta,int & Nctb,int & Nctc,
											 vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,vector<Elbow> & elb,vector<Sphere> & sph,vector<Body> & bd,vector<HollowBall> & hb,Contact *ct,Contact *cta,Contact *ctb,Contact *ctc,Data & dat,Gravity & gf,
											 Sphere *cell[], int & Ntp, char *name,bool record,int ntpi, int ntpf, double Gamma, double f, int Nthreshold) noexcept {
	record = 0;
	for(int nt = ntpi  ; nt <= ntpf ; nt++){
		//Secousse
		Secousse(pl,plr,co,Npl,Nplr,Nco,Gamma,f,dat);
		printf("Total = %e\n",dat.Total);
		Ntp = Evolution::EvolveOMP(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,Ncta,Nctb,Nctc,pl,plr,co,elb,sph,bd,hb,ct,cta,ctb,ctc,dat,gf,cell,Ntp, name,record,Nthreshold);
		// Relaxation
		Relaxation(pl,plr,co,Npl,Nplr,Nco,dat);
		printf("Total = %e\n",dat.Total);
		Ntp = Evolution::EvolveOMP(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,Ncta,Nctb,Nctc,pl,plr,co,elb,sph,bd,hb,ct,cta,ctb,ctc,dat,gf,cell,Ntp, name,record,Nthreshold);
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
			ReadWrite::writeOutData(name, nt, &gf, &dat);
			ReadWrite::writeOutHollowBall(name, Ntp, Nhb, hb);
			printf("Ntp = %d\n",nt);
			Ntp++;
		}
	}
	return Ntp;
}
