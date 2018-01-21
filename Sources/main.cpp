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
#include "../Includes/Data.h"
#include "../Includes/Move.h"
#include "../Includes/Evolution.h"
#include "../Includes/Compaction.h"
#include "../Includes/BodySpecie.h"
#include "../Includes/PowderPaQ.h"
#include "../Includes/HollowBall.h"
#include "../Includes/Option.h"
#include <iostream>
#include <vector>
//#include <omp.h>
#include "../Includes/AffinityCache.h"
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
	int Nsph0 = 0;
	double dila = 0;
	bool record = 1;
	int Nthreshold = 0;

	vector<Plan> pl;
	vector<PlanR> plr;
	vector<Cone> co;
	vector<Elbow> elb;
	vector<Sphere> sph;
	vector<BodySpecie> bdsp;
	vector<Body> bd;
	vector<HollowBall> hb;
	Option opt;

	Data dat;
	Gravity gf;

	printf("\n");

	if(opt.Management(argv,argc) == 0)
		return 0;

	if(opt.DirectoryManagement() == 0)
		return 0;

	opt.Record();

	if(opt.restart == 0){
		// Loading file from Export directory
		ReadWrite::readOutContainer(opt.directory,pl,plr,co,elb);
		ReadWrite::readOutSphere(opt.directory,sph,opt.limitNg);
		ReadWrite::readOutBodies(opt.directory,bd,opt.limitNbd);

		Nsph0 = sph.size();
		ReadWrite::readOutBodySpecie(opt.directory,bdsp);
		for( int i = 0 ; i < bd.size() ; i++)
			bd[i].UploadSpecies(bdsp,sph,i);

		ReadWrite::readOutData(opt.directory, &gf, &dat);
		ReadWrite::readOutHollowBall(opt.directory, hb);
	}
	else{
		// Loading file from Export directory
		ReadWrite::readStart_stopContainer(opt.directory,pl,plr,co,elb);
		ReadWrite::readStart_stopSphere(opt.directory,sph,opt.limitNg);
		ReadWrite::readStart_stopBodies(opt.directory,bd,opt.limitNbd);
		Nsph0 = sph.size();
		ReadWrite::readOutBodySpecie(opt.directory,bdsp);
		for(int i = 0 ; i < bd.size() ; i++)
			bd[i].UploadSpecies(bdsp,sph,i);

		ReadWrite::readStartStopData(opt.directory, &gf, &dat);
		ReadWrite::readStart_stopHollowBall(opt.directory, hb);
	}


	opt.InData(dat, gf, pl, plr, co);

	// Calcul du rayon maximum dans Data
	if(!sph.empty())
		dat.ComputeRmax(sph);

	// Calcul des cellules liees en fonction de dila et de Rmax dans Data
	if(dila != 0)
		dat.ComputeLinkedCell();

	// Annulation des vitesses
	if(opt.cancelVel == 1)
		CancelVelocity(sph,bd);
	// Generation de vitesse aleatoire
	if(opt.RdmVel == 1)
		RandomVelocity(sph,bd,opt.Vrdm,opt.Wrdm);

	// Gel des rotations
	if(opt.NoRotation == 1)
		FreezeRotation(bd);

	// Lien entre les spheres et les solides
	Sphere::sphereLinking(sph,  bd);


	for(auto& cone : co)
		cone.LimitLink(plr);

	// Lien entre les spheres et la hollowball dans laquelle elles sont
	for(int i = 0 ; i < hb.size() ; ++i)
		hb[i].Makeavatar(sph,i);

	for(int i = 0 ; i < hb.size() ; ++i)
		hb[i].LinkInSph(sph,i);


	for(auto& plan : pl)
		plan.InitList(sph.size());


	printf("Nsph0 = %d & Nsph = %d\n",Nsph0,static_cast<int>(sph.size()));

	printf("NctMax = %d\n",18*static_cast<int>(sph.size())+75*static_cast<int>(bd.size()));

	int Ncell = dat.Nx*dat.Ny*dat.Nz;
	dat.Ncellmax = Ncell;
	printf("Ncell = %d\n",Ncell);
	Sphere *cell[Ncell];

	printf("List of Linking Cell for Solid\n");
	printf("------------------------------\n\n");
	// Making list of linking cell for each solid
	ContactDetection::listCellForPlan(&dat, pl, gf);
	ContactDetection::listCellForPlanR(&dat, plr, gf);
	ContactDetection::listCellForCone(&dat, co, gf);
	ContactDetection::listCellForElbow(&dat, elb);
	printf("\n");

	if(dat.TIME != 0)
		Ntp = (int)((dat.TIME-dat.t0)/(dat.dts))+1;
	else
		Ntp = 0;

	if(opt.mode == 0){
		if(Ntp == 0 && dat.t0 <= dat.TIME){
			ReadWrite::writeStartStopContainer(opt.directory,pl,plr,co,elb);
			ReadWrite::writeStartStopSphere(opt.directory,sph);
			ReadWrite::writeStartStopBodies(opt.directory,bd,sph);
			ReadWrite::writeStartStopData(opt.directory, &gf, &dat);
			ReadWrite::writeStartStopHollowBall(opt.directory, hb);

			ReadWrite::writeOutContainer(opt.directory,Ntp,pl,plr,co,elb,dat.outMode);
			ReadWrite::writeOutSphere(opt.directory,Ntp,sph,dat.outMode);
			ReadWrite::writeOutBodies(opt.directory,Ntp,bd,dat.outMode);
			ReadWrite::writeOutData(opt.directory, Ntp, &gf, &dat);
			ReadWrite::writeOutHollowBall(opt.directory, Ntp, hb);
			Ntp++;
		}
	}

	if(opt.mode == 1 || opt.mode == 2){
		if(opt.NtapMin == 1){
			ReadWrite::writeStartStopContainer(opt.directory,pl,plr,co,elb);
			ReadWrite::writeStartStopSphere(opt.directory,sph);
			ReadWrite::writeStartStopBodies(opt.directory,bd,sph);
			ReadWrite::writeStartStopData(opt.directory, &gf, &dat);
			ReadWrite::writeStartStopHollowBall(opt.directory, hb);

			ReadWrite::writeOutContainer(opt.directory,Ntp,pl,plr,co,elb,dat.outMode);
			ReadWrite::writeOutSphere(opt.directory,Ntp,sph,dat.outMode);
			ReadWrite::writeOutBodies(opt.directory,Ntp,bd,dat.outMode);
			ReadWrite::writeOutHollowBall(opt.directory, Ntp, hb);
			ReadWrite::writeOutData(opt.directory, Ntp, &gf, &dat);
			Ntp++;
		}
	}
	printf("Time Path = %e\n\n",dat.dt);

	switch(opt.mode){
	case 0:
	{
		Evolution evolution(sph.size(), bd.size());
		Ntp = evolution.Evolve(pl,plr,co,elb,sph,bd,hb, dat,gf,cell,Ntp, opt.directory,record,Nthreshold);
		break;
	}
	case 1:
		dat.Total = 0;
		Ntp = Compaction::Run(pl,plr,co,elb,sph,bd,hb ,dat,gf,cell,Ntp, opt.directory,record,opt.NtapMin,opt.NtapMax,opt.Gamma,opt.Freq,Nthreshold);
		break;
	case 2:
		dat.Total = 0.0;
		Ntp = PowderPaQ::PowderPaQRun(pl,plr,co,elb,sph,bd,hb,dat,gf,cell,Ntp, opt.directory,record,opt.NtapMin,opt.NtapMax,Nthreshold,opt.PQheight,opt.PQVel);
		break;
	}


	for(int i = 0 ; i < bdsp.size() ; i++)
		bdsp[i].FreeMemory();


	if(opt.compression == 1){
		char commande[1024];
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
