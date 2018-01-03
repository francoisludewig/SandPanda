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
		Move::moveContainer(Npl, Nplr, Nco, Nelb, pl, plr, co, elb, dat.TIME, dat.dt/2, sph,gf);
		dat.mas->Move(dat.dt/2);
		Move::moveSphere(Nsph, sph, dat.dt/2);
		Move::upDateHollowBall(Nhb,hb,dat.dt);
		
		gf.Move(dat.TIME,dat.dt/2);
		//Doublon pour test
		Move::moveBodies(Nbd, bd, dat.dt/2, sph);
		PeriodicityPL(Nsph, Nbd, Npl, sph, bd, pl);
		
		// Linked Cells
		ContactDetection::linkedCell(sph,Nsph,&dat,cell);
		// Initialization for the time step
		ComputeForce::InitForTimeStep(Nsph, Nbd, Nct,Npl,Nplr,Nco,Nelb, sph, bd, ct,pl,plr,co,elb);
		// Contact Detection
		Nct = 0;
		// Verison sequentiel normale
		ContactDetection::sphContact(0, dat.Nx,dat.Nx,0, dat.Ny, dat.Ny, dat.Nz, ct, Nct, cell);
		ContactDetection::sphContainer(Nsph,Npl, Nplr, Nco, Nelb, Nhb, sph, pl, plr, co, elb, hb, Nct, ct, cell,dat.Rmax);
		
		if(dat.modelTg == 1){
			for(int i = 0 ; i < Nsph ; i++)
				sph[i].InitXsi();
				for(int i = 0 ; i < Nbd ; i++)
					bd[i].InitXsi();
					}
		
		// Computing Force
		ComputeForce::Compute(ct, Nct,dat);
		
		Move::UpDateForceContainer(Nsph,sph,Npl,Nplr,Nco,pl,plr,co,dat.TIME,dat.dt,gf);
		dat.mas->getForces();
		
		// Update Velocities
		Move::upDateVelocitySphere(Nsph, sph, gf, dat.dt);
		Move::upDateVelocityBodies(Nbd, bd, gf, dat.dt, sph);
		Move::upDateVelocityContainer(Npl, Nplr, Nco, Nelb, pl, plr, co, elb, dat.TIME, dat.dt, gf);
		dat.mas->UpDateVelocity(dat.dt);
		
		// Move
		Move::moveContainer(Npl, Nplr, Nco, Nelb, pl, plr, co, elb, dat.TIME, dat.dt/2, sph,gf);
		dat.mas->Move(dat.dt/2);
		Move::moveSphere(Nsph, sph, dat.dt/2);
		Move::moveBodies(Nbd, bd, dat.dt/2, sph);
		Move::upDateHollowBall(Nhb,hb,dat.dt);
		gf.Move(dat.TIME,dat.dt/2);
		PeriodicityPL(Nsph, Nbd, Npl, sph, bd, pl);
		
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


int Evolution::EvolveOMP(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,int & Ncta,int & Nctb,int & Nctc,
												 vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,vector<Elbow> & elb,vector<Sphere> & sph,vector<Body> & bd,vector<HollowBall> & hb,Contact *ct,Contact *cta,Contact *ctb,Contact *ctc,Data & dat,Gravity & gf,
												 Sphere *cell[], int & Ntp, char *name,bool record, int Nthreshold) noexcept {
	int Nprocess = 2;
	if(ctb != NULL)
		Nprocess = 4;
		// OMP Version
		do{
			dat.TIME += dat.dt;
			// Position anticipation
			Move::moveContainer(Npl, Nplr, Nco, Nelb, pl, plr, co, elb, dat.TIME, dat.dt/2, sph,gf);
			
			Move::moveSphereOMP(Nsph, sph, dat.dt/2,Nprocess);
			Move::moveBodiesOMP(Nbd, bd, dat.dt/2, sph,Nprocess);
			Move::upDateHollowBall(Nhb,hb,dat.dt);
			gf.Move(dat.TIME,dat.dt/2);
			PeriodicityPL(Nsph, Nbd, Npl, sph, bd, pl);
			
			// Linked Cells
			//linkedCell(sph, Nsph, &dat, tdl, cell);
			ContactDetection::linkedCell(sph,Nsph,&dat,cell);
			
			// Initialization for the time step
			ComputeForce::InitForTimeStepOMP(Nsph, Nbd, Nct,Ncta,Nctb,Nctc,Npl,Nplr,Nco,Nelb, sph, bd, ct, cta, ctb, ctc,pl,plr,co,elb);
			
			// Contact Detection
			Nct = 0;
			Ncta = 0;
			Nctb = 0;
			Nctc = 0;
			
			// Detection des contacts grain-grain
			if(Nprocess == 2){
#pragma omp parallel shared(cell)
				{
#pragma omp sections
					{
#pragma omp section
					ContactDetection::sphContact(0       , dat.Nx/2,dat.Nx,0, dat.Ny, dat.Ny, dat.Nz, ct , Nct , cell);
#pragma omp section
					ContactDetection::sphContact(dat.Nx/2, dat.Nx  ,dat.Nx,0, dat.Ny, dat.Ny, dat.Nz, cta, Ncta, cell);
					}
				}
			}
			else{
#pragma omp parallel shared(cell)
				{
#pragma omp sections
					{
#pragma omp section
					ContactDetection::sphContact(0       , dat.Nx/2, dat.Nx, 0       , dat.Ny/2, dat.Ny, dat.Nz, ct , Nct , cell);
#pragma omp section
					ContactDetection::sphContact(0       , dat.Nx/2, dat.Nx, dat.Ny/2, dat.Ny  , dat.Ny, dat.Nz, cta, Ncta, cell);
#pragma omp section
					ContactDetection::sphContact(dat.Nx/2, dat.Nx  , dat.Nx, 0       , dat.Ny/2, dat.Ny, dat.Nz, ctb, Nctb, cell);
#pragma omp section
					ContactDetection::sphContact(dat.Nx/2, dat.Nx  , dat.Nx, dat.Ny/2, dat.Ny  , dat.Ny, dat.Nz, ctc, Nctc, cell);
					}
				}
			}
			
			// Detection des contacts grain-container
			ContactDetection::sphContainerOMP(Nsph,Npl, Nplr, Nco, Nelb, Nhb, sph, pl, plr, co, elb, hb, Nct, ct, Ncta, cta, cell,dat.Rmax);
			
			
			if(dat.modelTg == 1){
				for(int i = 0 ; i < Nsph ; i++)
					sph[i].InitXsi();
					for(int i = 0 ; i < Nbd ; i++)
						bd[i].InitXsi();
						}
			
			
			
			// Rassemblement des contacts pour un calcul des forces sequentiel
			
			/*
			 for(int i = 0 ; i < Ncta ; i++){
			 ct[Nct] = cta[i];
			 Nct++;
			 }
			 if(Nprocess == 4){
			 for(int i = 0 ; i < Nctb ; i++){
			 ct[Nct] = ctb[i];
			 Nct++;
			 }
			 for(int i = 0 ; i < Nctc ; i++){
			 ct[Nct] = ctc[i];
			 Nct++;
			 }
			 }
			 */
			
			// Computing Force
			ComputeForce::Compute(ct, Nct,dat);
			ComputeForce::Compute(cta, Ncta,dat);
			if(Nprocess == 4){
				ComputeForce::Compute(ctb, Nctb,dat);
				ComputeForce::Compute(ctc, Nctc,dat);
			}
			
			Move::UpDateForceContainer(Nsph,sph,Npl,Nplr,Nco,pl,plr,co,dat.TIME,dat.dt,gf);
			
			// Update Velocities
			Move::upDateVelocitySphereOMP(Nsph, sph, gf, dat.dt,Nprocess);
			Move::upDateVelocityBodiesOMP(Nbd, bd, gf, dat.dt, sph,Nprocess);
			Move::upDateVelocityContainer(Npl, Nplr, Nco, Nelb, pl, plr, co, elb, dat.TIME, dat.dt, gf);
			
			// Move
			Move::moveContainer(Npl, Nplr, Nco, Nelb, pl, plr, co, elb, dat.TIME, dat.dt/2, sph,gf);
			Move::moveSphereOMP(Nsph, sph, dat.dt/2,Nprocess);
			Move::upDateHollowBall(Nhb,hb,dat.dt);
			
			Move::moveBodiesOMP(Nbd, bd, dat.dt/2, sph,Nprocess);
			gf.Move(dat.TIME,dat.dt/2);
			PeriodicityPL(Nsph, Nbd, Npl, sph, bd, pl);
			
			
			if(record){
				if(fabs((dat.TIME-dat.t0)-Ntp*(dat.dts)) < dat.dt  && dat.TIME-dat.t0 > 0.){
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
		}while(dat.TIME <= dat.Total-dat.dt);
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
		Move::moveContainer(Npl, Nplr, Nco, Nelb, pl, plr, co, elb, dat.TIME, dat.dt/2, sph,gf);
		Move::moveSphere(Nsph, sph, dat.dt/2);
		Move::upDateHollowBall(Nhb,hb,dat.dt);
		Move::MeltingSphere(Nsph, sph, vr, delayVr, dat.dt/2);
		gf.Move(dat.TIME,dat.dt/2);
		//Doublon pour test
		Move::moveBodies(Nbd, bd, dat.dt/2, sph);
		PeriodicityPL(Nsph, Nbd, Npl, sph, bd, pl);
		
		// Linked Cells
		ContactDetection::linkedCell(sph,Nsph,&dat,cell);
		// Initialization for the time step
		ComputeForce::InitForTimeStep(Nsph, Nbd, Nct,Npl,Nplr,Nco,Nelb, sph, bd, ct,pl,plr,co,elb);
		
		// Contact Detection
		Nct = 0;
		ContactDetection::sphContact(0, dat.Nx,dat.Nx,0, dat.Ny, dat.Ny, dat.Nz, ct, Nct, cell);
		ContactDetection::sphContainer(Nsph,Npl, Nplr, Nco, Nelb, Nhb, sph, pl, plr, co, elb, hb, Nct, ct, cell,dat.Rmax);
		
		if(dat.modelTg == 1){
			for(int i = 0 ; i < Nsph ; i++)
				sph[i].InitXsi();
				}
		//printf("Nct = %d\n",Nct);
		
		// Computing Force
		ComputeForce::Compute(ct, Nct,dat);
		
		// Update Velocities
		Move::upDateVelocitySphere(Nsph, sph, gf, dat.dt);
		Move::upDateVelocityBodies(Nbd, bd, gf, dat.dt, sph);
		// Move
		Move::moveContainer(Npl, Nplr, Nco, Nelb, pl, plr, co, elb, dat.TIME, dat.dt/2, sph,gf);
		Move::moveSphere(Nsph, sph, dat.dt/2);
		Move::moveBodies(Nbd, bd, dat.dt/2, sph);
		Move::upDateHollowBall(Nhb,hb,dat.dt);
		Move::MeltingSphere(Nsph, sph, vr, delayVr, dat.dt/2);
		gf.Move(dat.TIME,dat.dt/2);
		PeriodicityPL(Nsph, Nbd, Npl, sph, bd, pl);
		
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

int Evolution::EvolveMeltOMP(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,int & Ncta,int & Nctb,int & Nctc,
														 vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co,vector<Elbow> & elb,vector<Sphere> & sph,vector<Body> & bd,vector<HollowBall> & hb,Contact *ct,Contact *cta,Contact *ctb,Contact *ctc,Data & dat,Gravity & gf,
														 Sphere *cell[], int & Ntp, char *name,bool record, double vr,double delayVr, int Nthreshold) noexcept{
	int i;
	int Nprocess = 2;
	double dtl;
	double r0,r1;
	if(ctb != NULL)
		Nprocess = 4;
		// OMP Version
		do{
			dat.TIME += dat.dt;
			dtl = dat.dt;
			r0 = sph[0].Radius();
			
			// Position anticipation
			Move::moveContainer(Npl, Nplr, Nco, Nelb, pl, plr, co, elb, dat.TIME, dat.dt/2, sph,gf);
			Move::moveSphereOMP(Nsph, sph, dat.dt/2,Nprocess);
			Move::moveBodiesOMP(Nbd, bd, dat.dt/2, sph,Nprocess);
			Move::upDateHollowBall(Nhb,hb,dat.dt);
			
			Move::MeltingSphere(Nsph, sph, vr, delayVr, dat.dt/2);
			
			gf.Move(dat.TIME,dat.dt/2);
			PeriodicityPL(Nsph, Nbd, Npl, sph, bd, pl);
			
			// Linked Cells
			//linkedCell(sph, Nsph, &dat, tdl, cell);
			ContactDetection::linkedCell(sph,Nsph,&dat,cell);
			
			// Initialization for the time step
			ComputeForce::InitForTimeStep(Nsph, Nbd, Nct,Npl,Nplr,Nco,Nelb, sph, bd, ct,pl,plr,co,elb);
			
			// Contact Detection
			Nct = 0;
			Ncta = 0;
			Nctb = 0;
			Nctc = 0;
			// Detection des contacts grain-grain
			if(Nprocess == 2){
#pragma omp parallel shared(cell)
				{
#pragma omp sections
					{
#pragma omp section
					ContactDetection::sphContact(0       , dat.Nx/2,dat.Nx,0, dat.Ny, dat.Ny, dat.Nz, ct , Nct , cell);
#pragma omp section
					ContactDetection::sphContact(dat.Nx/2, dat.Nx  ,dat.Nx,0, dat.Ny, dat.Ny, dat.Nz, cta, Ncta, cell);
					}
				}
			}
			else{
#pragma omp parallel shared(cell)
				{
#pragma omp sections
					{
#pragma omp section
					ContactDetection::sphContact(0       , dat.Nx/2, dat.Nx, 0       , dat.Ny/2, dat.Ny, dat.Nz, ct , Nct , cell);
#pragma omp section
					ContactDetection::sphContact(0       , dat.Nx/2, dat.Nx, dat.Ny/2, dat.Ny  , dat.Ny, dat.Nz, cta, Ncta, cell);
#pragma omp section
					ContactDetection::sphContact(dat.Nx/2, dat.Nx  , dat.Nx, 0       , dat.Ny/2, dat.Ny, dat.Nz, ctb, Nctb, cell);
#pragma omp section
					ContactDetection::sphContact(dat.Nx/2, dat.Nx  , dat.Nx, dat.Ny/2, dat.Ny  , dat.Ny, dat.Nz, ctc, Nctc, cell);
					}
				}
			}
			
			for(i = 0 ; i < Ncta ; i++){
				ct[Nct] = cta[i];
				Nct++;
			}
			if(Nprocess == 4){
				for(i = 0 ; i < Nctb ; i++){
					ct[Nct] = ctb[i];
					Nct++;
				}
				for(i = 0 ; i < Nctc ; i++){
					ct[Nct] = ctc[i];
					Nct++;
				}
			}
			
			// Detection des contacts grain-container
			ContactDetection::sphContainer(Nsph,Npl, Nplr, Nco, Nelb, Nhb, sph, pl, plr, co, elb, hb, Nct, ct, cell,dat.Rmax);
			
			if(dat.modelTg == 1){
				for(int i = 0 ; i < Nsph ; i++)
					sph[i].InitXsi();
					}
			// Calcul des forces
			ComputeForce::Compute(ct,Nct,dat);
			
			// Update Velocities
			Move::upDateVelocitySphereOMP(Nsph, sph, gf, dat.dt,Nprocess);
			Move::upDateVelocityBodiesOMP(Nbd, bd, gf, dat.dt, sph,Nprocess);
			
			
			// Move
			Move::moveContainer(Npl, Nplr, Nco, Nelb, pl, plr, co, elb, dat.TIME, dat.dt/2, sph,gf);
			Move::moveSphereOMP(Nsph, sph, dat.dt/2,Nprocess);
			Move::moveBodiesOMP(Nbd, bd, dat.dt/2, sph,Nprocess);
			Move::upDateHollowBall(Nhb,hb,dat.dt);
			
			Move::MeltingSphere(Nsph, sph, vr, delayVr, dat.dt/2);
			gf.Move(dat.TIME,dat.dt/2);
			PeriodicityPL(Nsph, Nbd, Npl, sph, bd, pl);
			
			if(record){
				if(fabs((dat.TIME-dat.t0)-Ntp*(dat.dts)) < dat.dt  && dat.TIME-dat.t0 > 0.){
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
	return(Ntp);
}
