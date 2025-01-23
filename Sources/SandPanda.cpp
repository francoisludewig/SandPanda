#include "Velocity.h"
#include "Gravity.h"
#include "Plan.h"
#include "PlanR.h"
#include "Cone.h"
#include "Elbow.h"
#include "Sphere.h"
#include "Bodies.h"
#include "Contact.h"
#include "ReadWrite.h"
#include "ContactDetection.h"
#include "ComputingForce.h"
#include "Data.h"
#include "Move.h"
#include "Evolution.h"
#include "Compaction.h"
#include "BodySpecie.h"
#include "PowderPaQ.h"
#include "HollowBall.h"
#include "Option.h"
#include <iostream>
#include <vector>
//#include <omp.h>
#include "AffinityCache.h"
#include <sys/stat.h>

using namespace std;

void CancelVelocity(vector<Sphere> & sph, int & Nsph, vector<Bodies> & bd, int Nbd){
	for(int i = 0 ;  i < Nsph ; i++){
		sph[i].CancelVelocity();
	}
	for(int i = 0 ;  i < Nbd ; i++){
		bd[i].CancelVelocity();
	}
}

void RandomVelocity(vector<Sphere> & sph, int & Nsph, vector<Bodies> & bd, int Nbd, double V, double W){
	for(int i = 0 ;  i < Nsph ; i++){
		sph[i].RandomVelocity(V,W);
	}
	for(int i = 0 ;  i < Nbd ; i++){
		bd[i].RandomVelocity(V,W);
	}
}

void FreezeRotation(vector<Bodies> & bd, int Nbd){
	for(int i = 0 ;  i < Nbd ; i++){
		bd[i].setActiveRotation(1);
	}
}

void deborde(){
	printf("Memory Lacking\n");
	printf("Simulation is stopped\n");
	exit(-1);
}

int main(int argc,char **argv){
	int Ntp;
	int Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nct = 0,Ncta = 0,Nctb = 0,Nctc = 0,Nbdsp,Nhb;
	double dila = 0;
	bool record = 1;
	int Nthreshold = 0;
    
	vector<Plan> pl;
	vector<PlanR> plr;
	vector<Cone> co;
	vector<Elbow> elb;
	vector<Sphere> sph;
	vector<BodySpecie> bdsp;
	vector<Bodies> bd;
	vector<HollowBall> hb;
	Option opt;

	Contact *ct = NULL,*cta = NULL,*ctb = NULL,*ctc = NULL;
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
		readOutContainer(opt.directory,Npl,Nplr,Nco,Nelb,pl,plr,co,elb);
		readOutSphere(opt.directory,Nsph,sph,opt.limitNg);
		readOutBodies(opt.directory,Nbd,bd,opt.limitNbd);
		
		Nsph0 = Nsph;
		readOutBodySpecie(opt.directory,Nbdsp,bdsp);
		for(int i = 0 ; i < Nbd ; i++){
			bd[i].uploadSpecies(Nbdsp,bdsp,sph,Nsph,i);
		}
		readOutData(opt.directory, &gf, &dat);
        readOutHollowBall(opt.directory, Nhb, hb);
	}
	else{
		// Loading file from Export directory
		readStart_stopContainer(opt.directory,Npl,Nplr,Nco,Nelb,pl,plr,co,elb);
		readStart_stopSphere(opt.directory,Nsph,sph,opt.limitNg);
		readStart_stopBodies(opt.directory,Nbd,bd,opt.limitNbd);
		Nsph0 = Nsph;
		readOutBodySpecie(opt.directory,Nbdsp,bdsp);
		for(int i = 0 ; i < Nbd ; i++){
			bd[i].uploadSpecies(Nbdsp,bdsp,sph,Nsph,i);
		}
		readStartStopData(opt.directory, &gf, &dat);
        readStart_stopHollowBall(opt.directory, Nhb, hb);
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
	sphereLinking(Nsph, sph,  bd);
    
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
		pl[i].initList(Nsph);
	}
	
	printf("Nsph0 = %d & Nsph = %d\n",Nsph0,Nsph);
	
	printf("NctMax = %d\n",18*Nsph+75*Nbd);
	// Dynamical allocation
	switch(opt.parallel){
			case 0:
			ct  = new Contact[18*Nsph+75*Nbd];
			cta = NULL;
			ctb = NULL;
			ctc = NULL;
			break;
			case 1:
			switch(opt.Nprocess){
					case 2:
					ct  = new Contact[18*Nsph];
					cta = new Contact[9*Nsph];
					ctb = NULL;
					ctc = NULL;
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
					ctb = NULL;
					ctc = NULL;
					opt.Nprocess = 2;
					break;
			}
			break;
	}
	
	int Ncell = dat.Nx*dat.Ny*dat.Nz;
	dat.Ncellmax = Ncell;
	printf("Ncell = %d\n",Ncell);
	Sphere *cell[Ncell];
	
	
	printf("List of Linking Cell for Solid\n");
	printf("------------------------------\n\n");
	// Making list of linking cell for each solid
	listCellForPlan(&dat, pl, Npl, gf);
	listCellForPlanR(&dat, plr, Nplr, gf);
	listCellForCone(&dat, co, Nco, gf);
	listCellForElbow(&dat, elb, Nelb);
	printf("\n");
	// Control taille de la memoire demandee
	printf("Etat de la memoire\n");
	printf("------------------\n\n");
	int All = 0;
	All += Nsph*sizeof(Sphere);
	All += Nbdsp*sizeof(BodySpecie);
	All += Nbd*sizeof(Bodies);
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
			writeStartStopContainer(opt.directory,Npl,Nplr,Nco,Nelb,pl,plr,co,elb);
			writeStartStopSphere(opt.directory,Nsph,sph);
			writeStartStopBodies(opt.directory,Nbd,bd,sph);
			writeStartStopData(opt.directory, &gf, &dat);
			writeStartStopHollowBall(opt.directory, Nhb, hb);
            
			writeOutContainer(opt.directory,Ntp,Npl,Nplr,Nco,Nelb,pl,plr,co,elb,dat.outMode);
			writeOutSphere(opt.directory,Ntp,Nsph0,sph,dat.outMode);
			writeOutBodies(opt.directory,Ntp,Nbd,bd,dat.outMode);
			writeOutData(opt.directory, Ntp, &gf, &dat);
            writeOutHollowBall(opt.directory, Ntp, Nhb, hb);
			Ntp++;
		}
	}
	
	if(opt.mode == 1 || opt.mode == 2){
		if(opt.NtapMin == 1){
			writeStartStopContainer(opt.directory,Npl,Nplr,Nco,Nelb,pl,plr,co,elb);
			writeStartStopSphere(opt.directory,Nsph,sph);
			writeStartStopBodies(opt.directory,Nbd,bd,sph);
			writeStartStopData(opt.directory, &gf, &dat);
            writeStartStopHollowBall(opt.directory, Nhb, hb);
			
			writeOutContainer(opt.directory,Ntp,Npl,Nplr,Nco,Nelb,pl,plr,co,elb,dat.outMode);
			writeOutSphere(opt.directory,Ntp,Nsph0,sph,dat.outMode);
			writeOutBodies(opt.directory,Ntp,Nbd,bd,dat.outMode);
            writeOutHollowBall(opt.directory, Ntp, Nhb, hb);
			writeOutData(opt.directory, Ntp, &gf, &dat);
			Ntp++;
		}
	}
	printf("Time Path = %e\n\n",dat.dt);
	
    switch(opt.mode){
			case 0:
            if(opt.parallel == 1)
			Ntp = evolutionOMP(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,Ncta,Nctb,Nctc,pl,plr,co,elb,sph,bd,hb,ct,cta,ctb,ctc,dat,gf,cell,Ntp, opt.directory,record,Nthreshold);
            else
			Ntp = evolution(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,               pl,plr,co,elb,sph,bd,hb,ct,            dat,gf,cell,Ntp, opt.directory,record,Nthreshold);
            break;
			case 1:
            dat.Total = 0;
            if(opt.parallel == 0)
			Ntp = Compaction   (Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct               ,pl,plr,co,elb,sph,bd,hb,ct            ,dat,gf,cell,Ntp, opt.directory,record,opt.NtapMin,opt.NtapMax,opt.Gamma,opt.Freq,Nthreshold);
            else
			Ntp = CompactionOMP(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,Ncta,Nctb,Nctc,pl,plr,co,elb,sph,bd,hb,ct,cta,ctb,ctc,dat,gf,cell,Ntp, opt.directory,record,opt.NtapMin,opt.NtapMax,opt.Gamma,opt.Freq,Nthreshold);
            
            break;
			case 2:
            dat.Total = 0.0;
            if(opt.parallel == 0)
			Ntp = PowderPaQ   (Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct               ,pl,plr,co,elb,sph,bd,hb,ct            ,dat,gf,cell,Ntp, opt.directory,record,opt.NtapMin,opt.NtapMax,Nthreshold,opt.PQheight,opt.PQVel);
            else
			Ntp = PowderPaQOMP(Npl,Nplr,Nco,Nelb,Nsph,Nsph0,Nbd,Nhb,Nct,Ncta,Nctb,Nctc,pl,plr,co,elb,sph,bd,hb,ct,cta,ctb,ctc,dat,gf,cell,Ntp, opt.directory,record,opt.NtapMin,opt.NtapMax,Nthreshold,opt.PQheight,opt.PQVel);
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
	
	// Liberation manuelle de tableau
	for(int i = 0 ; i < Nsph ; i++){
		sph[i].SphDealloc();
	}
	
    
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
    
	return 0;
}
