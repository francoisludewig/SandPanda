#include "../../Includes/Dynamic/PowderPaQ.h"

#include "../../Includes/Solids/Velocity.h"
#include "../../Includes/Solids/Plan.h"
#include "../../Includes/Solids/PlanR.h"
#include "../../Includes/Solids/Cone.h"
#include "../../Includes/Solids/Elbow.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/Body.h"
#include "../../Includes/Repository/SimulationData.h"
#include "../../Includes/Repository/ReadWrite.h"
#include "../../Includes/ComputingForce.h"
#include "../../Includes/Dynamic/Evolution.h"
#include "../../Includes/Configuration/Configuration.h"
#include "../../Includes/Configuration/Monitoring.h"

void PowderPaQ::PowderPaQsecousseUpward(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Configuration & dat, double PQheight, double PQVel) noexcept {
	for(auto& plan : pl){
		plan.SetVz(PQVel,0,0,0);
		plan.SetMemoryPosition();
	}
	for(auto& disk : plr){
		disk.SetVz(PQVel,0,0,0);
		disk.SetMemoryPosition();
	}
	for(auto& cone : co){
		cone.SetVz(PQVel,0,0,0);
		cone.SetMemoryPosition();
	}		
	dat.Total += PQheight/PQVel;
}

void PowderPaQ::PowderPaQsecousseDownward(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Configuration & dat, double PQheight) noexcept {
	double delay = sqrt(2*PQheight/9.81);
	dat.Total += delay;	
	int Ntsp = (delay)/(dat.dt) + 1;
	dat.dt = (delay)/(double)(Ntsp);
	//printf("dt = %e\n",dat.dt);
	for(auto& plan : pl){
		plan.SetVz(0,0,0,0);
		plan.OnOffGravity(true);
		plan.SetVelocityToZero();
	}
	for(auto& disk : plr){
		disk.SetVz(0,0,0,0);
		disk.OnOffGravity(true);
		disk.SetVelocityToZero();
	}
	for(auto& cone : co){
		cone.SetVz(0,0,0,0);
		cone.OnOffGravity(true);
		cone.SetVelocityToZero();
	}	
}

void PowderPaQ::PowderPaQrelaxation(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Configuration & dat, double t, double PQheight, double PQVel) noexcept {
	for(auto& plan : pl){
		plan.SetVz(0,0,0,0);
		plan.OnOffGravity(false);
		plan.GetMemoryPosition();
	}
	for(auto& disk : plr){
		disk.SetVz(0,0,0,0);
		disk.OnOffGravity(false);
		disk.GetMemoryPosition();
	}
	for(auto& cone : co){
		cone.SetVz(0,0,0,0);
		cone.OnOffGravity(false);
		cone.GetMemoryPosition();
	}
	dat.Total += (t-PQheight/PQVel-sqrt(2*PQheight/9.81));	
}

int PowderPaQ::PowderPaQRun(std::shared_ptr<SimulationData>& solids, std::vector<Sphere*>& cell, int & Ntp, char *name, int ntpi, int ntpf, double PQheight, double PQVel, const CellBounds& cellBounds, bool
                            isMonitoringActivated) noexcept {
	Evolution evolution(solids, cellBounds, false);
	solids->configuration.record = 0;
	for(int nt = ntpi  ; nt <= ntpf ; nt++){
		//Secousse
		PowderPaQsecousseUpward(solids->plans,solids->disks,solids->cones,solids->configuration,PQheight,PQVel);
		Ntp = evolution.Evolve(cell,Ntp, name, false);

		PowderPaQsecousseDownward(solids->plans,solids->disks,solids->cones,solids->configuration,PQheight);
		Ntp = evolution.Evolve(cell,Ntp, name, false);

		// Relaxation
		if(nt < 100)
			PowderPaQrelaxation(solids->plans,solids->disks,solids->cones,solids->configuration,0.5,PQheight,PQVel);
		else
			PowderPaQrelaxation(solids->plans,solids->disks,solids->cones,solids->configuration,0.25,PQheight,PQVel);

		Ntp = evolution.Evolve(cell,Ntp, name,false);
		// Enregistrement
		ReadWrite::writeStartStopContainer(name,solids->plans,solids->disks,solids->cones,solids->elbows);
		ReadWrite::writeStartStopSphere(name,solids->spheres);
		ReadWrite::writeStartStopBodies(name,solids->bodies,solids->spheres);
		ReadWrite::writeStartStopData(name, solids->gravity, solids->configuration);
		ReadWrite::writeStartStopHollowBall(name, solids->hollowBalls);

		ReadWrite::writeOutContainer(name,nt,solids->plans,solids->disks,solids->cones,solids->elbows,solids->configuration.outMode);
		ReadWrite::writeOutSphere(name,nt,solids->spheres,solids->configuration.outMode);
		ReadWrite::writeOutBodies(name,nt,solids->bodies,solids->configuration.outMode);
		ReadWrite::writeOutData(name, nt, solids->gravity, solids->configuration);
		ReadWrite::writeOutContact(name,nt,evolution.ContactCount(),evolution.GetContacts(),solids->configuration);
		ReadWrite::writeOutHollowBall(name, Ntp, solids->hollowBalls);
		Ntp++;
		if (isMonitoringActivated) {
			Monitoring::getInstance().metrics(nt, ntpf);
		}
	}
	return Ntp;
}
