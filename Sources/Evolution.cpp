#include <ctime>

#include "../Includes/Evolution.h"
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
#include "../Includes/Periodicity.h"
#include "../Includes/ComputingForce.h"
#include "../Includes/Data.h"
#include "../Includes/Move.h"
#include "../Includes/HollowBall.h"

int Evolution::Evolve(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,
											vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,vector<Elbow> & elb,vector<Sphere> & sph,vector<Body> & bd,vector<HollowBall> & hb,Contact *ct,Data & dat,Gravity & gf,
											Sphere *cell[], int & Ntp, char *name,bool record, int Nthreshold) noexcept {
	// Sequential Version
	printf("Evolution\n");
	do{
		dat.TIME += dat.dt;
		// Position anticipation
		Move::moveContainer(pl, plr, co, elb, dat.TIME, dat.dt/2, sph,gf);
		dat.mas->Move(dat.dt/2);
		Move::moveSphere(sph, dat.dt/2);
		Move::upDateHollowBall(hb,dat.dt);
		
		gf.Move(dat.TIME,dat.dt/2);
		//Doublon pour test
		Move::moveBodies(bd, dat.dt/2, sph);
		PeriodicityPL(sph, pl);
		
		// Linked Cells
		ContactDetection::linkedCell(sph,&dat,cell);
		// Initialization for the time step
		ComputeForce::InitForTimeStep(Nct, sph, bd, ct,pl,plr,co,elb);
		// Contact Detection
		Nct = 0;
		// Verison sequentiel normale
		ContactDetection::sphContact(0, dat.Nx,dat.Nx,0, dat.Ny, dat.Ny, dat.Nz, ct, Nct, cell);
		ContactDetection::sphContainer(sph, pl, plr, co, elb, hb, Nct, ct, cell,dat.Rmax);
		
		if(dat.modelTg == 1){
			for(int i = 0 ; i < Nsph ; i++) {
				sph[i].GetElongationManager().InitXsi();
			}

				for(int i = 0 ; i < Nbd ; i++)
					bd[i].GetElongationManager().InitXsi();
					}
		
		// Computing Force
		ComputeForce::Compute(ct, Nct,dat);
		
		Move::UpDateForceContainer(sph,pl,plr,co,dat.TIME,dat.dt,gf);
		dat.mas->getForces();
		
		// Update Velocities
		Move::upDateVelocitySphere(sph, gf, dat.dt);
		Move::upDateVelocityBodies(bd, gf, dat.dt, sph);
		Move::upDateVelocityContainer(pl, plr, co, elb, dat.TIME, dat.dt, gf);
		dat.mas->UpDateVelocity(dat.dt);
		
		// Move
		Move::moveContainer(pl, plr, co, elb, dat.TIME, dat.dt/2, sph,gf);
		dat.mas->Move(dat.dt/2);
		Move::moveSphere(sph, dat.dt/2);
		Move::moveBodies(bd, dat.dt/2, sph);
		Move::upDateHollowBall(hb,dat.dt);
		gf.Move(dat.TIME,dat.dt/2);
		PeriodicityPL(sph, pl);
		
		// Record data
		if(record){
			if(fabs((dat.TIME-dat.t0)-Ntp*(dat.dts)) < dat.dt*0.99  && (dat.TIME-dat.t0 > 0.)){
				ReadWrite::writeStartStopContainer(name,Npl,Nplr,Nco,Nelb,pl,plr,co,elb);
				ReadWrite::writeStartStopSphere(name,Nsph,sph);
				ReadWrite::writeStartStopBodies(name,Nbd,bd,sph);
				ReadWrite::writeStartStopData(name, &gf, &dat);
				ReadWrite::writeStartStopHollowBall(name, Nhb, hb);
				
				ReadWrite::writeOutContainer(name,Ntp,Npl,Nplr,Nco,Nelb,pl,plr,co,elb,dat.outMode);
				ReadWrite::writeOutSphere(name,Ntp,Nsph0,sph,dat.outMode);
				ReadWrite::writeOutBodies(name,Ntp,Nbd,bd,dat.outMode);
				ReadWrite::writeOutHollowBall(name, Ntp, Nhb, hb);
				//writeOutData(name, Ntp, &gf, &dat);
				if(dat.outContact == 1 || dat.outContact > 2)
					ReadWrite::writeOutContact(name,Ntp,Nct,ct,dat);
					if(dat.outContact >= 2)
						ReadWrite::writeOutContactDetails(name,Ntp,Nct,ct,dat);
						printf("Save File %d\t\ttime = %e\r",Ntp,dat.TIME);
						fflush(stdout);
						Ntp++;
			}
		}
	}while(dat.TIME <= dat.Total-dat.dt*0.99);
	//printf("TIME = %.20lf > %.20lf = Total-dt\n",dat.TIME,dat.Total-dat.dt*0.9);
	printf("\n");
	return(Ntp);
}

int Evolution::EvolveMelt(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,
													vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,vector<Elbow> & elb,vector<Sphere> & sph,vector<Body> & bd,vector<HollowBall> & hb,Contact *ct,Data & dat,Gravity & gf,
													Sphere *cell[], int & Ntp, char *name,bool record, double vr,double delayVr, int Nthreshold) noexcept{
	printf("Evolution\n");
	double dtl;
	double r0,r1;
	// Sequential Version
	do{
		dat.TIME += dat.dt;
		dtl = dat.dt;
		r0 = sph[0].Radius();
		// Position anticipation
		Move::moveContainer(pl, plr, co, elb, dat.TIME, dat.dt/2, sph,gf);
		Move::moveSphere(sph, dat.dt/2);
		Move::upDateHollowBall(hb,dat.dt);
		Move::MeltingSphere(sph, vr, delayVr, dat.dt/2);
		gf.Move(dat.TIME,dat.dt/2);
		//Doublon pour test
		Move::moveBodies(bd, dat.dt/2, sph);
		PeriodicityPL(sph, pl);
		
		// Linked Cells
		ContactDetection::linkedCell(sph,&dat,cell);
		// Initialization for the time step
		ComputeForce::InitForTimeStep(Nct, sph, bd, ct,pl,plr,co,elb);
		
		// Contact Detection
		Nct = 0;
		ContactDetection::sphContact(0, dat.Nx,dat.Nx,0, dat.Ny, dat.Ny, dat.Nz, ct, Nct, cell);
		ContactDetection::sphContainer(sph, pl, plr, co, elb, hb, Nct, ct, cell,dat.Rmax);
		
		if(dat.modelTg == 1){
			for(int i = 0 ; i < Nsph ; i++) {
				sph[i].GetElongationManager().InitXsi();
				}
		}
		//printf("Nct = %d\n",Nct);
		
		// Computing Force
		ComputeForce::Compute(ct, Nct,dat);
		
		// Update Velocities
		Move::upDateVelocitySphere(sph, gf, dat.dt);
		Move::upDateVelocityBodies(bd, gf, dat.dt, sph);
		// Move
		Move::moveContainer(pl, plr, co, elb, dat.TIME, dat.dt/2, sph,gf);
		Move::moveSphere(sph, dat.dt/2);
		Move::moveBodies(bd, dat.dt/2, sph);
		Move::upDateHollowBall(hb,dat.dt);
		Move::MeltingSphere(sph, vr, delayVr, dat.dt/2);
		gf.Move(dat.TIME,dat.dt/2);
		PeriodicityPL(sph, pl);
		
		// Record data
		if(record){
			if(fabs((dat.TIME-dat.t0)-Ntp*(dat.dts)) < dat.dt  && (dat.TIME-dat.t0 > 0.)){
				ReadWrite::writeStartStopContainer(name,Npl,Nplr,Nco,Nelb,pl,plr,co,elb);
				ReadWrite::writeStartStopSphere(name,Nsph,sph);
				ReadWrite::writeStartStopBodies(name,Nbd,bd,sph);
				ReadWrite::writeStartStopData(name, &gf, &dat);
				ReadWrite::writeStartStopHollowBall(name, Nhb, hb);
				
				ReadWrite::writeOutContainer(name,Ntp,Npl,Nplr,Nco,Nelb,pl,plr,co,elb,dat.outMode);
				ReadWrite::writeOutSphere(name,Ntp,Nsph0,sph,dat.outMode);
				ReadWrite::writeOutBodies(name,Ntp,Nbd,bd,dat.outMode);
				ReadWrite::writeOutHollowBall(name, Ntp, Nhb, hb);
				if(dat.outContact == 1)
					ReadWrite::writeOutContact(name,Ntp,Nct,ct,dat);
					if(dat.outContact == 2)
						ReadWrite::writeOutContactDetails(name,Ntp,Nct,ct,dat);
						printf("Save File %d\t\ttime = %e\r",Ntp,dat.TIME);
						fflush(stdout);
						Ntp++;
			}
		}
		r1 = sph[0].Radius();
		dat.dt = dtl*pow(r1/r0,3./2.);
	}while(dat.TIME < dat.Total-dat.dt);
	printf("\n");
	return(Ntp);
}
