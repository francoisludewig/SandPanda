#include "../Includes/Solids/Velocity.h"
#include "../Includes/Configuration/Gravity.h"
#include "../Includes/Solids/Plan.h"
#include "../Includes/Solids/PlanR.h"
#include "../Includes/Solids/Cone.h"
#include "../Includes/Solids/Elbow.h"
#include "../Includes/Solids/Sphere.h"
#include "../Includes/Solids/Body.h"
#include "../Includes/Repository/ReadWrite.h"
#include "../Includes/ComputingForce.h"
#include "../Includes/Configuration/Configuration.h"
#include "../Includes/Dynamic/Move.h"
#include "../Includes/Dynamic/Evolution.h"
#include "../Includes/MultiThread.h"
#include "../Includes/Dynamic/Compaction.h"
#include "../Includes/Solids/BodySpecie.h"
#include "../Includes/Dynamic/PowderPaQ.h"
#include "../Includes/Solids/HollowBall.h"
#include "../Includes/Repository/SimulationData.h"
#include "../Includes/Repository/SimulationDataManager.h"
#include "../Includes/Configuration/Option.h"
#include "../Includes/Contact/ContactDetection.h"
#include "../Includes/LinkedCells/SolidCellsBuilder.h"
#include <iostream>
#include <vector>
#include <memory>
//#include <omp.h>
//#include "../Includes/AffinityCache.h"
#include <sys/stat.h>
#include <chrono>

using namespace std;

class TimeDurationMeasure {
public:
	const uint64_t nanoInMinute = 60L*1000000000L;
	const uint64_t nanoInSecond = 1000000000L;
	TimeDurationMeasure() {}
	~TimeDurationMeasure() = default;

	void Start() { t1 = std::chrono::high_resolution_clock::now(); }
	void Stop() {
		std::chrono::high_resolution_clock::time_point  t2 = std::chrono::high_resolution_clock::now();
		delay = std::chrono::duration_cast<std::chrono::nanoseconds>( t2 - t1 ).count();
	}

	std::string Nanosecond() { return std::to_string(delay); }
	std::string Minutes() { return std::to_string(delay/nanoInMinute) + "m" + std::to_string((delay%nanoInMinute)/nanoInSecond) + "s" + std::to_string(delay%nanoInSecond) + "ns"; }

private:

	std::chrono::high_resolution_clock::time_point t1;
	uint64_t delay {0};
};

void CancelVelocity(vector<Sphere> & sph, vector<Body> & bd) {
	for(auto& sphere : sph)
		sphere.CancelVelocity();

	for(auto& body : bd)
		body.CancelVelocity();
}

void RandomVelocity(vector<Sphere> & sph, vector<Body> & bd, double V, double W){
	for(auto& sphere : sph)
		sphere.RandomVelocity(V,W);

	for(auto& body : bd)
		body.RandomVelocity(V,W);

}

void FreezeRotation(vector<Body> & bd){
	for(auto& body : bd)
		body.SetActiveRotation(1);
}

void deborde(){
	printf("Memory Lacking\n");
	printf("Simulation is stopped\n");
	exit(-1);
}


int main(int argc,char **argv){
	TimeDurationMeasure tm;
	tm.Start();
	int Ntp;
	double dila = 0;
	int Nthreshold = 0;

	Option opt;

	printf("\n");

	if(opt.Management(argv,argc) == 0)
		return 0;

	if(opt.DirectoryManagement() == 0)
		return 0;

	opt.Record();


	std::shared_ptr<SimulationData> solids;

	if(opt.restart == 0)
		solids = SimulationDataManager::FromExport(opt);
	else
		solids = SimulationDataManager::FromStart_Stop(opt);

	opt.InData(solids->configuration, solids->gravity, solids->plans, solids->disks, solids->cones);

	// Calcul du rayon maximum dans Data
	if(!solids->spheres.empty())
		solids->configuration.ComputeRmax(solids->spheres);

	// Calcul des cellules liees en fonction de dila et de Rmax dans Data
	if(dila != 0)
		solids->configuration.ComputeLinkedCell();

	// Annulation des vitesses
	if(opt.cancelVel == 1)
		CancelVelocity(solids->spheres, solids->bodies);
	// Generation de vitesse aleatoire
	if(opt.RdmVel == 1)
		RandomVelocity(solids->spheres,solids->bodies,opt.Vrdm,opt.Wrdm);

	// Gel des rotations
	if(opt.NoRotation == 1)
		FreezeRotation(solids->bodies);

	// Lien entre les spheres et les solides
	Sphere::sphereLinking(solids->spheres,  solids->bodies);


	for(auto& cone : solids->cones)
		cone.LimitLink(solids->disks);

	// Lien entre les spheres et la hollowball dans laquelle elles sont
	for(int i = 0 ; i < solids->hollowBalls.size() ; ++i)
		solids->hollowBalls[i].Makeavatar(solids->spheres,i);

	for(int i = 0 ; i < solids->hollowBalls.size() ; ++i)
		solids->hollowBalls[i].LinkInSph(solids->spheres,i);


	for(auto& plan : solids->plans)
		plan.InitList(solids->spheres.size());


	//printf("Nsph0 = %d & Nsph = %d\n",Nsph0,static_cast<int>(sph.size()));

	//printf("NctMax = %d\n",18*static_cast<int>(sph.size())+75*static_cast<int>(bd.size()));

	int Ncell = solids->configuration.Nx*solids->configuration.Ny*solids->configuration.Nz;
	solids->configuration.Ncellmax = Ncell;
	printf("Ncell = %d\n",Ncell);
	std::vector<Sphere*> cell(Ncell, nullptr);
/*
	printf("List of Linking Cell for Solid\n");
	printf("------------------------------\n\n");
	// Making list of linking cell for each solid
	LinkedCellSolidListBuilder::ListCellForPlan(dat, pl, gf);
	LinkedCellSolidListBuilder::ListCellForPlanR(dat, plr, gf);
	LinkedCellSolidListBuilder::ListCellForCone(dat, co, gf);
	LinkedCellSolidListBuilder::ListCellForElbow(dat, elb);
	printf("\n");
*/
	solids->configuration.record = true;

	if(solids->configuration.TIME != 0)
		Ntp = (int)((solids->configuration.TIME-solids->configuration.t0)/(solids->configuration.dts))+1;
	else
		Ntp = 0;

	if(opt.mode == 0){
		if(Ntp == 0 && solids->configuration.t0 <= solids->configuration.TIME){
			SimulationDataManager::WriteStart_Stop(opt, solids, Ntp);
			Ntp++;
		}
	}

	if(opt.mode == 1 || opt.mode == 2){
		if(opt.NtapMin == 1){
			SimulationDataManager::WriteStart_Stop(opt, solids, Ntp);
			Ntp++;
		}
	}
	printf("Time Path = %e\n\n",solids->configuration.dt);


	CellBounds cellBounds(0, 0, 0, solids->configuration.Nx, solids->configuration.Ny, solids->configuration.Nz, 0, 0, 0, solids->configuration.Nx, solids->configuration.Ny, solids->configuration.Nz, solids->configuration.ax, solids->configuration.ay, solids->configuration.az, solids->configuration.xmin, solids->configuration.ymin, solids->configuration.zmin);
/*
	MultiThread mutlithread(4, sph.size(), bd.size(), pl, plr, co, elb, dat, gf, cellBounds);
	mutlithread.Run(pl,plr,co,elb,sph,bd,hb,dat,gf,cell,Ntp,opt.directory,Nthreshold);
*/

	switch(opt.mode){
	case 0:
	{
		Evolution evolution(solids, cellBounds, false);
		Ntp = evolution.Evolve(cell,Ntp, opt.directory,Nthreshold);
		break;
	}
	case 1:
		solids->configuration.Total = 0;
		Ntp = Compaction::Run(solids,cell,Ntp, opt.directory,opt.NtapMin,opt.NtapMax,opt.Gamma,opt.Freq,Nthreshold, cellBounds);
		break;
	case 2:
		solids->configuration.Total = 0.0;
		Ntp = PowderPaQ::PowderPaQRun(solids,cell,Ntp, opt.directory,opt.NtapMin,opt.NtapMax,Nthreshold,opt.PQheight,opt.PQVel, cellBounds);
		break;
	}

	if(opt.compression == 1){
		char commande[4096];
		sprintf(commande,"tar -zcvmf %s/Out.tgz %s/Out",opt.directory,opt.directory);
		system(commande);
		sprintf(commande,"tar -zcvmf %s/Start_stop.tgz %s/Start_stop",opt.directory,opt.directory);
		system(commande);
		sprintf(commande,"tar -zcvmf %s/Export.tgz %s/Export",opt.directory,opt.directory);
		system(commande);
		sprintf(commande,"rm -rf %s/Out %s/Start_stop %s/Export",opt.directory,opt.directory,opt.directory);
		system(commande);
	}

	tm.Stop();

	printf("Time: %s ns\n", tm.Minutes().c_str());
	printf("Time: %s ns\n", tm.Nanosecond().c_str());

	return 0;
}
