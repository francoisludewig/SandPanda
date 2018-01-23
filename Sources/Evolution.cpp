#include <ctime>

#include "../Includes/Evolution.h"
#include "../Includes/Data.h"
#include "../Includes/ComputingForce.h"
#include "../Includes/Move.h"
#include "../Includes/ReadWrite.h"
#include "../Includes/Contact/ContactDetection.h"
#include "../Includes/Periodicity.h"
#include "../Includes/Gravity.h"
#include "../Includes/Solids/Sphere.h"
#include "../Includes/Solids/Body.h"
#include "../Includes/LinkedCells/LinkedCellFiller.h"

int Evolution::Evolve(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Data & dat,Gravity & gf,
		std::vector<Sphere*>& cell, int & Ntp, char *name,bool record, int Nthreshold) noexcept {
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
		LinkedCellFiller::Fill(sph,dat,cell);
		// Initialization for the time step
		ComputeForce::InitForTimeStep(Nct, sph, bd, ct,pl,plr,co,elb);
		// Contact Detection
		Nct = 0;
		// Verison sequentiel normale
		ContactDetection::sphContact(0, dat.Nx,dat.Nx,0, dat.Ny, dat.Ny, dat.Nz, ct, Nct, cell);
		ContactDetection::sphContainer(sph, pl, plr, co, elb, hb, Nct, ct, cell,dat.Rmax);

		if(dat.modelTg == 1){
			for(auto& sphere : sph)
				sphere.GetElongationManager().InitXsi();

			for(auto& body : bd)
				body.GetElongationManager().InitXsi();
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
				ReadWrite::writeStartStopContainer(name,pl,plr,co,elb);
				ReadWrite::writeStartStopSphere(name,sph);
				ReadWrite::writeStartStopBodies(name,bd,sph);
				ReadWrite::writeStartStopData(name, &gf, &dat);
				ReadWrite::writeStartStopHollowBall(name, hb);

				ReadWrite::writeOutContainer(name,Ntp,pl,plr,co,elb,dat.outMode);
				ReadWrite::writeOutSphere(name,Ntp,sph,dat.outMode);
				ReadWrite::writeOutBodies(name,Ntp,bd,dat.outMode);
				ReadWrite::writeOutHollowBall(name, Ntp, hb);
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

int Evolution::EvolveMelt(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Data & dat,Gravity & gf,
		std::vector<Sphere*> cell, int & Ntp, char *name,bool record, double vr,double delayVr, int Nthreshold) noexcept{
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
		LinkedCellFiller::Fill(sph,dat,cell);
		// Initialization for the time step
		ComputeForce::InitForTimeStep(Nct, sph, bd, ct,pl,plr,co,elb);

		// Contact Detection
		Nct = 0;
		ContactDetection::sphContact(0, dat.Nx,dat.Nx,0, dat.Ny, dat.Ny, dat.Nz, ct, Nct, cell);
		ContactDetection::sphContainer(sph, pl, plr, co, elb, hb, Nct, ct, cell,dat.Rmax);

		if(dat.modelTg == 1){
			for(int i = 0 ; i < sph.size() ; i++) {
				sph[i].GetElongationManager().InitXsi();
			}
		}

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
				ReadWrite::writeStartStopContainer(name,pl,plr,co,elb);
				ReadWrite::writeStartStopSphere(name,sph);
				ReadWrite::writeStartStopBodies(name,bd,sph);
				ReadWrite::writeStartStopData(name, &gf, &dat);
				ReadWrite::writeStartStopHollowBall(name, hb);

				ReadWrite::writeOutContainer(name,Ntp,pl,plr,co,elb,dat.outMode);
				ReadWrite::writeOutSphere(name,Ntp,sph,dat.outMode);
				ReadWrite::writeOutBodies(name,Ntp,bd,dat.outMode);
				ReadWrite::writeOutHollowBall(name, Ntp, hb);
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
