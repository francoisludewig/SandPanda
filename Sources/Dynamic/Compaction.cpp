#include "../../Includes/Dynamic/Compaction.h"

#include "../../Includes/Solids/Velocity.h"
#include "../../Includes/Configuration/Gravity.h"
#include "../../Includes/Solids/Plan.h"
#include "../../Includes/Solids/PlanR.h"
#include "../../Includes/Solids/Cone.h"
#include "../../Includes/Solids/Elbow.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/Body.h"
#include "../../Includes/Repository/SimulationData.h"
#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Repository/ReadWrite.h"
#include "../../Includes/Contact/ContactDetection.h"
#include "../../Includes/ComputingForce.h"
#include "../../Includes/Configuration/Configuration.h"
#include "../../Includes/Dynamic/Move.h"
#include "../../Includes/Dynamic/Evolution.h"
#include "../../Includes/Solids/HollowBall.h"
#include "../../Includes/LinkedCells/CellBounds.h"

void Compaction::Secousse(vector<Plan> & pl,vector<PlanR> & plr,vector<Cone> & co, double Gamma, double f, Configuration & dat) noexcept {
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

void Compaction::Relaxation(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Configuration & dat) noexcept {
	for(auto& plan : pl)
		plan.SetVz(0,0,0,0);

	for(auto& disk : plr)
		disk.SetVz(0,0,0,0);

	for(auto& cone : co)
		cone.SetVz(0,0,0,0);

	dat.Total += 0.5;
}


int Compaction::Run(std::shared_ptr<SimulationData>& solids,std::vector<Sphere*>& cell, int & Ntp, char *name,int ntpi, int ntpf, double Gamma, double f, int Nthreshold, const CellBounds& cellBounds) noexcept {

	Evolution evolution(solids, cellBounds, false);
	solids->configuration.record = 0;
	for(int nt = ntpi  ; nt <= ntpf ; nt++){
		//Secousse
		Secousse(solids->plans,solids->disks,solids->cones,Gamma,f,solids->configuration);
		printf("Total = %e\n",solids->configuration.Total);
		Ntp = evolution.Evolve(cell,Ntp, name,Nthreshold);
		// Relaxation
		Relaxation(solids->plans,solids->disks,solids->cones,solids->configuration);
		printf("Total = %e\n",solids->configuration.Total);
		Ntp = evolution.Evolve(cell,Ntp, name,Nthreshold);

		if(solids->configuration.record == 0){
			// Enregistrement
			ReadWrite::writeStartStopContainer(name,solids->plans,solids->disks,solids->cones,solids->elbows);
			ReadWrite::writeStartStopSphere(name,solids->spheres);
			ReadWrite::writeStartStopBodies(name,solids->bodies,solids->spheres);
			ReadWrite::writeStartStopData(name, solids->gravity, solids->configuration);
			ReadWrite::writeStartStopHollowBall(name, solids->hollowBalls);

			ReadWrite::writeOutContainer(name,nt,solids->plans,solids->disks,solids->cones,solids->elbows,solids->configuration.outMode);
			ReadWrite::writeOutSphere(name,nt,solids->spheres,solids->configuration.outMode);
			ReadWrite::writeOutBodies(name,nt,solids->bodies,solids->configuration.outMode);
			//writeOutData(name, nt, &gf, &dat);
			ReadWrite::writeOutHollowBall(name, Ntp, solids->hollowBalls);
			printf("Ntp = %d\n",nt);
			Ntp++;
		}
	}
	return Ntp;
}
