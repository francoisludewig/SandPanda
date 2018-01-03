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

void PowderPaQ::PowderPaQsecousseUpward(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,int Npl, int Nplr, int Nco, Data & dat, double PQheight, double PQVel) noexcept {
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

void PowderPaQ::PowderPaQsecousseDownward(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,int Npl, int Nplr, int Nco, Data & dat, double PQheight) noexcept {
	int i;	
	double delay = sqrt(2*PQheight/9.81);
	dat.Total += delay;	
	int Ntsp = (delay)/(dat.dt) + 1;
	dat.dt = (delay)/(double)(Ntsp);
	//printf("dt = %e\n",dat.dt);
	for(i = 0 ; i < Npl ; i++){
		pl[i].SetVz(0,0,0,0);
		pl[i].OnOffGravity(true);
		pl[i].SetVelocityToZero();
	}
	for(i = 0 ; i < Nplr ; i++){
		plr[i].SetVz(0,0,0,0);
		plr[i].OnOffGravity(true);
		plr[i].SetVelocityToZero();
	}
	for(i = 0 ; i < Nco ; i++){
		co[i].SetVz(0,0,0,0);
		co[i].OnOffGravity(true);
		co[i].SetVelocityToZero();
	}	
}

void PowderPaQ::PowderPaQrelaxation(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,int Npl, int Nplr, int Nco, Data & dat, double t, double PQheight, double PQVel) noexcept {
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
		std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Contact *ct,Data & dat,Gravity & gf,
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
		std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Contact *ct,Contact *cta,Contact *ctb,Contact *ctc,Data & dat,Gravity & gf,
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

