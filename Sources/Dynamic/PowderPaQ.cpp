#include "../../Includes/Dynamic/PowderPaQ.h"

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

void PowderPaQ::PowderPaQsecousseUpward(vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,int Npl, int Nplr, int Nco, Data & dat, double PQheight, double PQVel) noexcept {
	int i;		
	for(i = 0 ; i < Npl ; i++){
		pl[i].SetVz(PQVel,0,0,0);
		pl[i].SetMemoryPosition();
	}
	for(i = 0 ; i < Nplr ; i++){
		plr[i].SetVz(PQVel,0,0,0);
		plr[i].SetMemoryPosition();
	}
	for(i = 0 ; i < Nco ; i++){
		co[i].SetVz(PQVel,0,0,0);
		co[i].SetMemoryPosition();
		
	}		
	dat.Total += PQheight/PQVel;	
	//int Ntsp = (0.03)/(dat.dt);
	//dat.dt = dat.Total/Ntsp;
}

void PowderPaQ::PowderPaQsecousseDownward(vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,int Npl, int Nplr, int Nco, Data & dat, double PQheight) noexcept {
	int i;	
	double delay = sqrt(2*PQheight/9.81);
	dat.Total += delay;	
	int Ntsp = (delay)/(dat.dt) + 1;
	dat.dt = (delay)/(double)(Ntsp);
	//printf("dt = %e\n",dat.dt);
	for(i = 0 ; i < Npl ; i++){
		pl[i].SetVz(0,0,0,0);
		pl[i].OnOffGravity(true);
        pl[i].resetVelocities();
	}
	for(i = 0 ; i < Nplr ; i++){
		plr[i].SetVz(0,0,0,0);
		plr[i].OnOffGravity(true);
        plr[i].resetVelocities();
	}
	for(i = 0 ; i < Nco ; i++){
		co[i].SetVz(0,0,0,0);
		co[i].OnOffGravity(true);
        co[i].resetVelocities();
	}	
}

void PowderPaQ::PowderPaQrelaxation(vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,int Npl, int Nplr, int Nco, Data & dat, double t, double PQheight, double PQVel) noexcept {
	int i;
	for(i = 0 ; i < Npl ; i++){
		pl[i].SetVz(0,0,0,0);
		pl[i].OnOffGravity(false);
		pl[i].GetMemoryPosition();
	}
	for(i = 0 ; i < Nplr ; i++){
		plr[i].SetVz(0,0,0,0);
		plr[i].OnOffGravity(false);
		plr[i].GetMemoryPosition();
	}
	for(i = 0 ; i < Nco ; i++){
		co[i].SetVz(0,0,0,0);
		co[i].OnOffGravity(false);
		co[i].GetMemoryPosition();
	}
	dat.Total += (t-PQheight/PQVel-sqrt(2*PQheight/9.81));	
}

int PowderPaQ::PowderPaQRun(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,
														vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,vector<Elbow> & elb,vector<Sphere> & sph,vector<Body> & bd,vector<HollowBall> & hb,Contact *ct,Data & dat,Gravity & gf,
														Sphere *cell[], int & Ntp, char *name,bool record,int ntpi, int ntpf, int Nthreshold, double PQheight, double PQVel) noexcept {
	record = 0;
	for(int nt = ntpi  ; nt <= ntpf ; nt++){
		//Secousse
		PowderPaQsecousseUpward(pl,plr,co,Npl,Nplr,Nco,dat,PQheight,PQVel);
		Ntp = Evolution::Evolve(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,pl,plr,co,elb,sph,bd,hb,ct,dat,gf,cell,Ntp, name,record,Nthreshold);
		
		PowderPaQsecousseDownward(pl,plr,co,Npl,Nplr,Nco,dat,PQheight);
		Ntp = Evolution::Evolve(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,pl,plr,co,elb,sph,bd,hb,ct,dat,gf,cell,Ntp, name,record,Nthreshold);
		
		// Relaxation
		if(nt < 100)
			PowderPaQrelaxation(pl,plr,co,Npl,Nplr,Nco,dat,0.5,PQheight,PQVel);
			else
				PowderPaQrelaxation(pl,plr,co,Npl,Nplr,Nco,dat,0.25,PQheight,PQVel);
				
				Ntp = Evolution::Evolve(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,pl,plr,co,elb,sph,bd,hb,ct,dat,gf,cell,Ntp, name,record,Nthreshold);
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
		ReadWrite::writeOutContact(name,nt,Nct,ct,dat);
		ReadWrite::writeOutHollowBall(name, Ntp, Nhb, hb);
		Ntp++;
		
	}
	return Ntp;
}

int PowderPaQ::PowderPaQOMP(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,int & Ncta,int & Nctb,int & Nctc,
														vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,vector<Elbow> & elb,vector<Sphere> & sph,vector<Body> & bd,vector<HollowBall> & hb,Contact *ct,Contact *cta,Contact *ctb,Contact *ctc,Data & dat,Gravity & gf,
														Sphere *cell[], int & Ntp, char *name,bool record,int ntpi, int ntpf, int Nthreshold, double PQheight, double PQVel) noexcept {
	record = 0;
	for(int nt = ntpi  ; nt <= ntpf ; nt++){
		//Secousse
		PowderPaQsecousseUpward(pl,plr,co,Npl,Nplr,Nco,dat,PQheight,PQVel);
		printf("Total = %e\n",dat.Total);
		Ntp = Evolution::EvolveOMP(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,Ncta,Nctb,Nctc,pl,plr,co,elb,sph,bd,hb,ct,cta,ctb,ctc,dat,gf,cell,Ntp, name,record,Nthreshold);
		PowderPaQsecousseDownward(pl,plr,co,Npl,Nplr,Nco,dat,PQheight);
		printf("Total = %e\n",dat.Total);
		Ntp = Evolution::EvolveOMP(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,Ncta,Nctb,Nctc,pl,plr,co,elb,sph,bd,hb,ct,cta,ctb,ctc,dat,gf,cell,Ntp, name,record,Nthreshold);
		// Relaxation		
		if(nt < 100)
			PowderPaQrelaxation(pl,plr,co,Npl,Nplr,Nco,dat,0.5,PQheight,PQVel);
			else
				PowderPaQrelaxation(pl,plr,co,Npl,Nplr,Nco,dat,0.25,PQheight,PQVel);
				
				printf("Total = %e\n",dat.Total);
				Ntp = Evolution::EvolveOMP(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,Ncta,Nctb,Nctc,pl,plr,co,elb,sph,bd,hb,ct,cta,ctb,ctc,dat,gf,cell,Ntp, name,record,Nthreshold);
				// Enregistrement		
				ReadWrite::writeStartStopContainer(name,Npl,Nplr,Nco,Nelb,pl,plr,co,elb);
		ReadWrite::writeStartStopSphere(name,Nsph,sph);
		ReadWrite::writeStartStopBodies(name,Nbd,bd,sph);
		ReadWrite::writeStartStopData(name, &gf, &dat);
		ReadWrite::writeStartStopHollowBall(name, Nhb, hb);
		
		ReadWrite::writeOutContainer(name,nt,Npl,Nplr,Nco,Nelb,pl,plr,co,elb,dat.outMode);
		ReadWrite::writeOutSphere(name,nt,Nsph0,sph,dat.outMode
															);	
		ReadWrite::writeOutBodies(name,nt,Nbd,bd,dat.outMode);
		ReadWrite::writeOutData(name, nt, &gf, &dat);
		ReadWrite::writeOutContact(name,nt,Nct,ct,dat);
		ReadWrite::writeOutHollowBall(name, Ntp, Nhb, hb);
		printf("Ntp = %d\n",nt);
		Ntp++;
	}	
	return Ntp;
}

