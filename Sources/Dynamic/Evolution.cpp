#include <ctime>

#include "../../Includes/Dynamic/Evolution.h"
#include "../../Includes/ComputingForce.h"
#include "../../Includes/Configuration/Configuration.h"
#include "../../Includes/Dynamic/Move.h"
#include "../../Includes/Repository/ReadWrite.h"
#include "../../Includes/Contact/ContactDetection.h"
#include "../../Includes/Dynamic/Periodicity.h"
#include "../../Includes/Configuration/Gravity.h"
#include "../../Includes/Configuration/Monitoring.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/Body.h"
#include "../../Includes/LinkedCells/LinkedCellFiller.h"

int Evolution::Evolve(std::vector<Sphere*>& cell, int & Ntp, char *name, const bool isMonitoringActivated) noexcept {
	double dt = solids->configuration.dt;

	// Sequential Version
	printf("Evolution\n");
	do{
		solids->configuration.TIME += dt;
		// Position anticipation
		Move::moveContainer(solids->plans, solids->disks, solids->cones, solids->elbows, solids->configuration.TIME, dt/2, solids->spheres,solids->gravity);
		solids->configuration.mas->Move(dt/2);

		Move::moveSphere(solids->spheres, dt/2);
		Move::upDateHollowBall(solids->hollowBalls, dt);

		solids->gravity.Move(solids->configuration.TIME,dt/2);
		//Doublon pour test
		Move::moveBodies(solids->bodies, dt/2, solids->spheres);
		PeriodicityPL(solids->spheres, solids->plans);

		// Linked Cells
		LinkedCellFiller::Fill(solids->spheres,solids->configuration,cell);
		// Initialization for the time step
		ComputeForce::InitForTimeStep(Nct, solids->spheres, solids->bodies, ct,solids->plans,solids->disks,solids->cones,solids->elbows);
		// Contact Detection
		Nct = 0;
		// Verison sequentiel normale
		ContactDetection::sphContact(cellBounds, ct, Nct, cell);
		ContactDetection::sphContainer(solids->spheres, solids->plans, solids->disks, solids->cones, solids->elbows, solids->hollowBalls, Nct, ct, cell, solidCells,solids->configuration.Rmax);

		if(solids->configuration.modelTg == 1){
			for(auto& sphere : solids->spheres)
				sphere.GetElongationManager().InitXsi();

			for(auto& body : solids->bodies)
				body.GetElongationManager().InitXsi();
		}

		// Computing Force
		if(isMultiThreads)
			ComputeForce::ComputeMutex(ct, Nct,solids->configuration);
		else
			ComputeForce::Compute(ct, Nct,solids->configuration);

		Move::UpDateForceContainer(solids->spheres,solids->plans,solids->disks,solids->cones,solids->configuration.TIME, dt, solids->gravity);
		solids->configuration.mas->getForces();
		// Update Velocities
		Move::upDateVelocitySphere(solids->spheres, solids->gravity, dt);
		Move::upDateVelocityBodies(solids->bodies, solids->gravity, dt, solids->spheres);
		Move::upDateVelocityContainer(solids->plans, solids->disks, solids->cones, solids->elbows, solids->configuration.TIME, dt, solids->gravity);
		solids->configuration.mas->UpDateVelocity(dt);

		// Move
		Move::moveContainer(solids->plans, solids->disks, solids->cones, solids->elbows, solids->configuration.TIME, dt/2, solids->spheres, solids->gravity);
		solids->configuration.mas->Move(dt/2);
		Move::moveSphere(solids->spheres, dt/2);
		Move::moveBodies(solids->bodies, dt/2, solids->spheres);
		Move::upDateHollowBall(solids->hollowBalls,dt);

		solids->gravity.Move(solids->configuration.TIME,dt/2);
		PeriodicityPL(solids->spheres, solids->plans);

		// Record data
		if(solids->configuration.record){
			if(fabs((solids->configuration.TIME-solids->configuration.t0)-Ntp*(solids->configuration.dts)) < solids->configuration.dt*0.99  && (solids->configuration.TIME-solids->configuration.t0 > 0.)){

				ReadWrite::writeStartStopContainer(name,solids->plans,solids->disks,solids->cones,solids->elbows);
				ReadWrite::writeStartStopSphere(name,solids->spheres);
				ReadWrite::writeStartStopBodies(name,solids->bodies,solids->spheres);
				ReadWrite::writeStartStopData(name, solids->gravity, solids->configuration);
				ReadWrite::writeStartStopHollowBall(name, solids->hollowBalls);

				ReadWrite::writeOutContainer(name,Ntp,solids->plans,solids->disks,solids->cones,solids->elbows,solids->configuration.outMode);
				ReadWrite::writeOutSphere(name,Ntp,solids->spheres,solids->configuration.outMode);
				ReadWrite::writeOutBodies(name,Ntp,solids->bodies,solids->configuration.outMode);
				ReadWrite::writeOutHollowBall(name, Ntp, solids->hollowBalls);

				//writeOutData(name, Ntp, &gf, &dat);
				if(solids->configuration.outContact == 1 || solids->configuration.outContact > 2)
					ReadWrite::writeOutContact(name,Ntp,Nct,ct,solids->configuration);
				if(solids->configuration.outContact >= 2)
					ReadWrite::writeOutContactDetails(name,Ntp,Nct,ct,solids->configuration);
				printf("Save File %d\t\ttime = %e\r",Ntp,solids->configuration.TIME);
				fflush(stdout);
				Ntp++;
				if (isMonitoringActivated) {
					Monitoring::getInstance().metrics(solids->configuration.TIME, solids->configuration.Total);
				}
			}
		}
	}while(solids->configuration.TIME <= solids->configuration.Total-solids->configuration.dt*0.99);
	//printf("TIME = %.20lf > %.20lf = Total-dt\n",dat.TIME,dat.Total-dat.dt*0.9);
	printf("\n");
	return(Ntp);
}

int Evolution::EvolveMelt(std::vector<Sphere*> cell, int & Ntp, char *name, double vr, double delayVr, const bool isMonitoringActivated) noexcept{
	double dt = solids->configuration.dt;
	printf("Evolution\n");
	double dtl;
	double r0,r1;
	// Sequential Version
	do{
		solids->configuration.TIME += dt;
		dtl = dt;
		r0 = solids->spheres[0].Radius();
		// Position anticipation
		Move::moveContainer(solids->plans, solids->disks, solids->cones, solids->elbows, solids->configuration.TIME, dt/2, solids->spheres,solids->gravity);
		Move::moveSphere(solids->spheres, dt/2);
		Move::moveBodies(solids->bodies, dt/2, solids->spheres);

		Move::upDateHollowBall(solids->hollowBalls, dt);
		Move::MeltingSphere(solids->spheres, vr, delayVr, dt/2);
		solids->gravity.Move(solids->configuration.TIME,dt/2);


		//Doublon pour test
		Move::moveBodies(solids->bodies, dt/2, solids->spheres);
		PeriodicityPL(solids->spheres, solids->plans);

		// Linked Cells
		LinkedCellFiller::Fill(solids->spheres,solids->configuration,cell);
		// Initialization for the time step
		ComputeForce::InitForTimeStep(Nct, solids->spheres, solids->bodies, ct,solids->plans,solids->disks,solids->cones,solids->elbows);
		// Contact Detection
		Nct = 0;
		// Verison sequentiel normale
		ContactDetection::sphContact(cellBounds, ct, Nct, cell);
		ContactDetection::sphContainer(solids->spheres, solids->plans, solids->disks, solids->cones, solids->elbows, solids->hollowBalls, Nct, ct, cell, solidCells,solids->configuration.Rmax);


		if(solids->configuration.modelTg == 1){
			for(auto& sphere : solids->spheres)
				sphere.GetElongationManager().InitXsi();
		}

		// Computing Force
		if(isMultiThreads)
			ComputeForce::ComputeMutex(ct, Nct,solids->configuration);
		else
			ComputeForce::Compute(ct, Nct,solids->configuration);


		// Update Velocities
		Move::upDateVelocitySphere(solids->spheres, solids->gravity, dt);
		Move::upDateVelocityBodies(solids->bodies, solids->gravity, dt, solids->spheres);
		Move::upDateVelocityContainer(solids->plans, solids->disks, solids->cones, solids->elbows, solids->configuration.TIME, dt, solids->gravity);
		solids->configuration.mas->UpDateVelocity(dt);


		// Move
		Move::moveContainer(solids->plans, solids->disks, solids->cones, solids->elbows, solids->configuration.TIME, dt/2, solids->spheres,solids->gravity);
		Move::moveSphere(solids->spheres, dt/2);
		Move::moveBodies(solids->bodies, dt/2, solids->spheres);


		Move::upDateHollowBall(solids->hollowBalls,dt);
		Move::MeltingSphere(solids->spheres, vr, delayVr, dt/2);

		solids->gravity.Move(solids->configuration.TIME,dt/2);
		PeriodicityPL(solids->spheres, solids->plans);

		// Record data
		if(solids->configuration.record){
			if(fabs((solids->configuration.TIME-solids->configuration.t0)-Ntp*(solids->configuration.dts)) < solids->configuration.dt*0.99  && (solids->configuration.TIME-solids->configuration.t0 > 0.)){

				ReadWrite::writeStartStopContainer(name,solids->plans,solids->disks,solids->cones,solids->elbows);
				ReadWrite::writeStartStopSphere(name,solids->spheres);
				ReadWrite::writeStartStopBodies(name,solids->bodies,solids->spheres);
				ReadWrite::writeStartStopData(name, solids->gravity, solids->configuration);
				ReadWrite::writeStartStopHollowBall(name, solids->hollowBalls);

				ReadWrite::writeOutContainer(name,Ntp,solids->plans,solids->disks,solids->cones,solids->elbows,solids->configuration.outMode);
				ReadWrite::writeOutSphere(name,Ntp,solids->spheres,solids->configuration.outMode);
				ReadWrite::writeOutBodies(name,Ntp,solids->bodies,solids->configuration.outMode);
				ReadWrite::writeOutHollowBall(name, Ntp, solids->hollowBalls);

				//writeOutData(name, Ntp, &gf, &dat);
				if(solids->configuration.outContact == 1 || solids->configuration.outContact > 2)
					ReadWrite::writeOutContact(name,Ntp,Nct,ct,solids->configuration);
				if(solids->configuration.outContact >= 2)
					ReadWrite::writeOutContactDetails(name,Ntp,Nct,ct,solids->configuration);
				printf("Save File %d\t\ttime = %e\r",Ntp,solids->configuration.TIME);
				fflush(stdout);
				Ntp++;
				if (isMonitoringActivated) {
					Monitoring::getInstance().metrics(solids->configuration.TIME, solids->configuration.Total);
				}
			}
		}
		r1 = solids->spheres[0].Radius();
		solids->configuration.dt = dtl*pow(r1/r0,3./2.);
		dt = dtl*pow(r1/r0,3./2.);
	}while(solids->configuration.TIME <= solids->configuration.Total-solids->configuration.dt*0.99);
	printf("\n");
	printf("End of Evolution\n");
	return(Ntp);
}
