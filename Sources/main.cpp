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

void CancelVelocity(vector<Sphere> & sph, int & Nsph, vector<Body> & bd, int Nbd){
	for(int i = 0 ;  i < Nsph ; i++){
		sph[i].CancelVelocity();
	}
	for(int i = 0 ;  i < Nbd ; i++){
		bd[i].CancelVelocity();
	}
}

void RandomVelocity(vector<Sphere> & sph, int & Nsph, vector<Body> & bd, int Nbd, double V, double W){
	for(int i = 0 ;  i < Nsph ; i++){
		sph[i].RandomVelocity(V,W);
	}
	for(int i = 0 ;  i < Nbd ; i++){
		bd[i].RandomVelocity(V,W);
	}
}

void FreezeRotation(vector<Body> & bd, int Nbd){
	for(int i = 0 ;  i < Nbd ; i++){
		bd[i].SetActiveRotation(1);
	}
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
	int Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nct = 0,Nbdsp,Nhb;
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

	Contact *ct = nullptr;
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
		ReadWrite::readOutContainer(opt.directory,Npl,Nplr,Nco,Nelb,pl,plr,co,elb);
		ReadWrite::readOutSphere(opt.directory,Nsph,sph,opt.limitNg);
		ReadWrite::readOutBodies(opt.directory,Nbd,bd,opt.limitNbd);
		
		Nsph0 = Nsph;
		ReadWrite::readOutBodySpecie(opt.directory,Nbdsp,bdsp);
		for(int i = 0 ; i < Nbd ; i++){
			bd[i].UploadSpecies(Nbdsp,bdsp,sph,Nsph,i);
		}
		ReadWrite::readOutData(opt.directory, &gf, &dat);
		ReadWrite::readOutHollowBall(opt.directory, Nhb, hb);
	}
	else{
		// Loading file from Export directory
		ReadWrite::readStart_stopContainer(opt.directory,Npl,Nplr,Nco,Nelb,pl,plr,co,elb);
		ReadWrite::readStart_stopSphere(opt.directory,Nsph,sph,opt.limitNg);
		ReadWrite::readStart_stopBodies(opt.directory,Nbd,bd,opt.limitNbd);
		Nsph0 = Nsph;
		ReadWrite::readOutBodySpecie(opt.directory,Nbdsp,bdsp);
		for(int i = 0 ; i < Nbd ; i++){
			bd[i].UploadSpecies(Nbdsp,bdsp,sph,Nsph,i);
		}
		ReadWrite::readStartStopData(opt.directory, &gf, &dat);
		ReadWrite::readStart_stopHollowBall(opt.directory, Nhb, hb);
	}
	

	opt.InData(dat, gf, pl, plr, co);
	
	// Calcul du rayon maximum dans Data
	if(Nsph != 0)
		dat.ComputeRmax(sph,Nsph);
	
	// Calcul des cellules liees en fonction de dila et de Rmax dans Data
	if(dila != 0)
		dat.ComputeLinkedCell();
	
	// Annulation des vitesses
	if(opt.cancelVel == 1){
		CancelVelocity(sph,Nsph,bd,Nbd);
	}
	// Generation de vitesse aleatoire
	if(opt.RdmVel == 1){
		RandomVelocity(sph,Nsph,bd,Nbd,opt.Vrdm,opt.Wrdm);
		
	}
	// Gel des rotations
	if(opt.NoRotation == 1){
		FreezeRotation(bd,Nbd);
	}
	
	// Lien entre les spheres et les solides
	Sphere::sphereLinking(Nsph, sph,  bd);
	

	for(int i = 0 ; i < Nco ; i++){
		co[i].LimitLink(plr);
	}
	// Lien entre les spheres et la hollowball dans laquelle elles sont
	for(int i = 0 ; i < Nhb ; i++){
		hb[i].Makeavatar(sph,Nsph,i);
	}
	for(int i = 0 ; i < Nhb ; i++){
		hb[i].LinkInSph(sph,Nsph,i);
	}
	
	for(int i = 0 ; i < Npl ; i++){
		pl[i].InitList(Nsph);
	}
	
	printf("Nsph0 = %d & Nsph = %d\n",Nsph0,Nsph);
	
	printf("NctMax = %d\n",18*Nsph+75*Nbd);
	// Dynamical allocation
	ct  = new Contact[18*Nsph+75*Nbd];

	int Ncell = dat.Nx*dat.Ny*dat.Nz;
	dat.Ncellmax = Ncell;
	printf("Ncell = %d\n",Ncell);
	Sphere *cell[Ncell];
	
	printf("List of Linking Cell for Solid\n");
	printf("------------------------------\n\n");
	// Making list of linking cell for each solid
	ContactDetection::listCellForPlan(&dat, pl, Npl, gf);
	ContactDetection::listCellForPlanR(&dat, plr, Nplr, gf);
	ContactDetection::listCellForCone(&dat, co, Nco, gf);
	ContactDetection::listCellForElbow(&dat, elb, Nelb);
	printf("\n");
	/*
	sph.shrink_to_fit();
	pl.shrink_to_fit();
	plr.shrink_to_fit();
	co.shrink_to_fit();
	elb.shrink_to_fit();
	sph.shrink_to_fit();
	bdsp.shrink_to_fit();
	bd.shrink_to_fit();
	hb.shrink_to_fit();
*/
	// Control taille de la memoire demandee
	printf("Etat de la memoire\n");
	printf("------------------\n\n");
	int All = 0;
	All += Nsph*sizeof(Sphere);
	All += Nbdsp*sizeof(BodySpecie);
	All += Nbd*sizeof(Body);
	All += Npl*sizeof(Plan);
	All += Nplr*sizeof(PlanR);
	All += Nco*sizeof(Cone);
	All += Nelb*sizeof(Elbow);
	All += Nhb*sizeof(HollowBall);
	All += (18*Nsph+75*Nbd)*sizeof(Contact);
	All += sizeof(cell);
	All += sizeof(dat);
	All += sizeof(Gravity);
	
	printf("Le programme a alloue %f ko\n",All/1000.);
	printf("Le programme a alloue %f Mo\n",All/1000./1000.);
	
	if(dat.TIME != 0)
		Ntp = (int)((dat.TIME-dat.t0)/(dat.dts))+1;
	else
		Ntp = 0;
	
	if(opt.mode == 0){
		if(Ntp == 0 && dat.t0 <= dat.TIME){
			ReadWrite::writeStartStopContainer(opt.directory,Npl,Nplr,Nco,Nelb,pl,plr,co,elb);
			ReadWrite::writeStartStopSphere(opt.directory,Nsph,sph);
			ReadWrite::writeStartStopBodies(opt.directory,Nbd,bd,sph);
			ReadWrite::writeStartStopData(opt.directory, &gf, &dat);
			ReadWrite::writeStartStopHollowBall(opt.directory, Nhb, hb);
			
			ReadWrite::writeOutContainer(opt.directory,Ntp,Npl,Nplr,Nco,Nelb,pl,plr,co,elb,dat.outMode);
			ReadWrite::writeOutSphere(opt.directory,Ntp,Nsph0,sph,dat.outMode);
			ReadWrite::writeOutBodies(opt.directory,Ntp,Nbd,bd,dat.outMode);
			ReadWrite::writeOutData(opt.directory, Ntp, &gf, &dat);
			ReadWrite::writeOutHollowBall(opt.directory, Ntp, Nhb, hb);
			Ntp++;
		}
	}
	
	if(opt.mode == 1 || opt.mode == 2){
		if(opt.NtapMin == 1){
			ReadWrite::writeStartStopContainer(opt.directory,Npl,Nplr,Nco,Nelb,pl,plr,co,elb);
			ReadWrite::writeStartStopSphere(opt.directory,Nsph,sph);
			ReadWrite::writeStartStopBodies(opt.directory,Nbd,bd,sph);
			ReadWrite::writeStartStopData(opt.directory, &gf, &dat);
			ReadWrite::writeStartStopHollowBall(opt.directory, Nhb, hb);
			
			ReadWrite::writeOutContainer(opt.directory,Ntp,Npl,Nplr,Nco,Nelb,pl,plr,co,elb,dat.outMode);
			ReadWrite::writeOutSphere(opt.directory,Ntp,Nsph0,sph,dat.outMode);
			ReadWrite::writeOutBodies(opt.directory,Ntp,Nbd,bd,dat.outMode);
			ReadWrite::writeOutHollowBall(opt.directory, Ntp, Nhb, hb);
			ReadWrite::writeOutData(opt.directory, Ntp, &gf, &dat);
			Ntp++;
		}
	}
	printf("Time Path = %e\n\n",dat.dt);
	
	switch(opt.mode){
		case 0:
			Ntp = Evolution::Evolve(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,               pl,plr,co,elb,sph,bd,hb,ct,            dat,gf,cell,Ntp, opt.directory,record,Nthreshold);
			break;
		case 1:
			dat.Total = 0;
			Ntp = Compaction::Run(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct               ,pl,plr,co,elb,sph,bd,hb,ct            ,dat,gf,cell,Ntp, opt.directory,record,opt.NtapMin,opt.NtapMax,opt.Gamma,opt.Freq,Nthreshold);
			break;
		case 2:
			dat.Total = 0.0;
			Ntp = PowderPaQ::PowderPaQRun(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct               ,pl,plr,co,elb,sph,bd,hb,ct            ,dat,gf,cell,Ntp, opt.directory,record,opt.NtapMin,opt.NtapMax,Nthreshold,opt.PQheight,opt.PQVel);
			break;
	}
	
	// Free of dynamical table
	delete [] ct;
	
	for(int i = 0 ; i < Nbdsp ; i++){
		bdsp[i].FreeMemory();
	}
	
	pl.clear();
	plr.clear();
	co.clear();
	elb.clear();
	sph.clear();
	bd.clear();
	bdsp.clear();
	hb.clear();
	
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
