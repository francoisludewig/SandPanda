#include "../Includes/Object/Velocity.h"
#include "../Includes/Configuration/Gravity.h"
#include "../Includes/Object/Plan.h"
#include "../Includes/Object/PlanR.h"
#include "../Includes/Object/Cone.h"
#include "../Includes/Object/Elbow.h"
#include "../Includes/Object/Sphere.h"
#include "../Includes/Object/Body.h"
#include "../Includes/Repository/ReadWrite.h"
#include "../Includes/Configuration//Data.h"
#include "../Includes/Dynamic/Evolution.h"
#include "../Includes/Dynamic/Compaction.h"
#include "../Includes/Dynamic/PowderPaQ.h"
#include "../Includes/Configuration/Option.h"
#include "../Includes/Configuration/Monitoring.h"
#include <iostream>
#include <vector>
#include <filesystem>

using namespace std;

void CancelVelocity(vector<Sphere> & sph, int & Nsph, vector<Body> & bd, int Nbd){
	for(int i = 0 ;  i < Nsph ; i++){
        sph[i].resetVelocities();
	}
	for(int i = 0 ;  i < Nbd ; i++){
        bd[i].resetVelocities();
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

int main(int argc,char **argv){
	int Ntp;
	int Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nct = 0,Ncta = 0,Nctb = 0,Nctc = 0,Nbdsp,Nhb;
	//double dila = 0;
	bool record = true;
	int Nthreshold = 0;
    clock_t start = clock();

    vector<Plan> pl;
	vector<PlanR> plr;
	vector<Cone> co;
	vector<Elbow> elb;
	vector<Sphere> sph;
	vector<BodySpecie> bdsp;
	vector<Body> bd;
	vector<HollowBall> hb;
	Option opt;
	
	Contact *ct = nullptr,*cta = nullptr,*ctb = nullptr,*ctc = nullptr;
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
			bd[i].UploadSpecies(bdsp,sph,Nsph,i);
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
			bd[i].UploadSpecies(bdsp,sph,Nsph,i);
		}
		ReadWrite::readStartStopData(opt.directory, &gf, &dat);
		ReadWrite::readStart_stopHollowBall(opt.directory, Nhb, hb);
	}
	
	opt.InData(dat, gf, pl, plr, co);
	
	// Calcul du rayon maximum dans Data
	if(Nsph != 0)
		dat.ComputeRmax(sph,Nsph);
	
	/*
    // Calcul des cellules liees en fonction de dila et de Rmax dans Data
	if(dila != 0)
		dat.ComputeLinkedCell();
	*/

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

    for(auto& plan : pl)
        plan.InitList(Nsph);

	for(int i = 0 ; i < Npl ; i++){
		pl[i].InitList(Nsph);
	}

	printf("Nsph0 = %d & Nsph = %d\n",Nsph0,Nsph);
	
	printf("NctMax = %d\n",18*Nsph+75*Nbd);
	// Dynamical allocation
	switch(opt.parallel){
		case 0:
			ct  = new Contact[18*Nsph+75*Nbd];
			cta = nullptr;
			ctb = nullptr;
			ctc = nullptr;
			break;
		case 1:
			switch(opt.Nprocess){
				case 2:
					ct  = new Contact[18*Nsph];
					cta = new Contact[9*Nsph];
					ctb = nullptr;
					ctc = nullptr;
					break;
				case 4:
					ct  = new Contact[20*Nsph];
					cta = new Contact[5*Nsph];
					ctb = new Contact[5*Nsph];
					ctc = new Contact[5*Nsph];
					break;
				default:
					opt.Nprocess = 2;
					printf("Nprocess must be 2 or 4\nThe value used in this simulation will be 2\n");
					ct  = new Contact[18*Nsph];
					cta = new Contact[9*Nsph];
					ctb = nullptr;
					ctc = nullptr;
					opt.Nprocess = 2;
					break;
			}
			break;
	}
	
	int Ncell = dat.Nx*dat.Ny*dat.Nz;
	dat.Ncellmax = Ncell;
	printf("Ncell = %d\n",Ncell);
    // TODO Remettre dans la stack au lieu du heap pour gagner en performance !!!
    Sphere **cell = (Sphere**)malloc(sizeof(Sphere*)*Ncell);
	//Sphere *cell[Ncell];
	
	
	printf("List of Linking Cell for Solid\n");
	printf("------------------------------\n\n");
    fflush(stdout);
	// Making list of linking cell for each solid
	ContactDetection::listCellForPlan(&dat, pl, Npl, gf);
	ContactDetection::listCellForPlanR(&dat, plr, Nplr, gf);
	ContactDetection::listCellForCone(&dat, co, Nco, gf);
	ContactDetection::listCellForElbow(&dat, elb, Nelb);
	printf("\n");
	// Control taille de la memoire demandee
	printf("Etat de la memoire\n");
	printf("------------------\n\n");
	int All = 0;
	All += Nsph*static_cast<int>(sizeof(Sphere));
	All += Nbdsp*static_cast<int>(sizeof(BodySpecie));
	All += Nbd*static_cast<int>(sizeof(Body));
	All += Npl*static_cast<int>(sizeof(Plan));
	All += Nplr*static_cast<int>(sizeof(PlanR));
	All += Nco*static_cast<int>(sizeof(Cone));
	All += Nelb*static_cast<int>(sizeof(Elbow));
	All += Nhb*static_cast<int>(sizeof(HollowBall));
	All += (18*Nsph+75*Nbd)*static_cast<int>(sizeof(Contact));
	All += static_cast<int>(sizeof(cell));
	All += static_cast<int>(sizeof(dat));
	All += static_cast<int>(sizeof(Gravity));
	
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

    if(opt.isMonitoringActivated)
        Monitoring::getInstance().initialize(reinterpret_cast<string &>(opt.processName));

	printf("Time Path = %e\n\n",dat.dt);

	switch(opt.mode){
		case 0:
			if(opt.parallel == 1)
				Ntp = Evolution::EvolveOMP(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,Ncta,Nctb,Nctc,pl,plr,co,elb,sph,bd,hb,ct,cta,ctb,ctc,dat,gf,cell,Ntp, opt.directory,record,Nthreshold, opt.isMonitoringActivated);
			else
				Ntp = Evolution::Evolve(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,               pl,plr,co,elb,sph,bd,hb,ct,            dat,gf,cell,Ntp, opt.directory,record,Nthreshold, opt.isMonitoringActivated);
			break;
		case 1:
            dat.Total = dat.TIME;
			if(opt.parallel == 0)
				Ntp = Compaction::Run(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct               ,pl,plr,co,elb,sph,bd,hb,ct            ,dat,gf,cell,Ntp, opt.directory,record,opt.NtapMin,opt.NtapMax,opt.Gamma,opt.Freq,Nthreshold, opt.isMonitoringActivated);
			else
				Ntp = Compaction::RunOMP(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,Ncta,Nctb,Nctc,pl,plr,co,elb,sph,bd,hb,ct,cta,ctb,ctc,dat,gf,cell,Ntp, opt.directory,record,opt.NtapMin,opt.NtapMax,opt.Gamma,opt.Freq,Nthreshold, opt.isMonitoringActivated);
			
			break;
		case 2:
			dat.Total = dat.TIME;
			if(opt.parallel == 0)
				Ntp = PowderPaQ::PowderPaQRun(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct               ,pl,plr,co,elb,sph,bd,hb,ct            ,dat,gf,cell,Ntp, opt.directory,record,opt.NtapMin,opt.NtapMax,Nthreshold,opt.PQheight,opt.PQVel);
			else
				Ntp = PowderPaQ::PowderPaQOMP(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,Ncta,Nctb,Nctc,pl,plr,co,elb,sph,bd,hb,ct,cta,ctb,ctc,dat,gf,cell,Ntp, opt.directory,record,opt.NtapMin,opt.NtapMax,Nthreshold,opt.PQheight,opt.PQVel);
			break;
	}
	
	// Free of dynamical table
	delete [] ct;
	// OMP Vestion
	if(opt.parallel == 1){
		if(opt.Nprocess == 2)
			delete [] cta;
		else{
			delete [] ctb;
			delete [] ctc;
		}

	}

    free(cell);

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
        //TODO Use a library for archive in all operating systems
		sprintf(commande,"tar -zcvmf %s/Out.tgz %s/Out",opt.directory,opt.directory);
		system(commande);
		sprintf(commande,"tar -zcvmf %s/Start_stop.tgz %s/Start_stop",opt.directory,opt.directory);
		system(commande);
		sprintf(commande,"tar -zcvmf %s/Export.tgz %s/Export",opt.directory,opt.directory);
		system(commande);

        char directory[2048];
        sprintf(directory,"%s/Export", opt.directory);
        std::filesystem::remove_all(directory);

        sprintf(directory,"%s/Start_stop", opt.directory);
        std::filesystem::remove_all(directory);

        sprintf(directory,"%s/Out", opt.directory);
        std::filesystem::remove_all(directory);
		//sprintf(commande,"rm -rf %s/Out %s/Start_stop %s/Export",opt.directory,opt.directory,opt.directory);
		//system(commande);
	}
    clock_t stop = clock();
    double elapsed = (double) (stop - start) / CLOCKS_PER_SEC;
    printf("\nTime elapsed: %.5f\n", elapsed);
	return 0;
}
