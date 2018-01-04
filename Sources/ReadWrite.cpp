#include "../Includes/ReadWrite.h"

#include "../Includes/Velocity.h"
#include "../Includes/Gravity.h"
#include "../Includes/Data.h"
#include "../Includes/Plan.h"
#include "../Includes/PlanR.h"
#include "../Includes/Cone.h"
#include "../Includes/Elbow.h"
#include "../Includes/Sphere.h"
#include "../Includes/Body.h"
#include "../Includes/BodySpecie.h"
#include "../Includes/Contact.h"
#include "../Includes/Data.h"
#include "../Includes/HollowBall.h"

void ReadWrite::readOutBodySpecie(char *directory, int & Nbdsp,vector<BodySpecie> & bdsp) noexcept {
	char fileName[1024];
	FILE *ft;
	sprintf(fileName,"%sExport/particleSpecies.txt",directory);
	printf("Uploading ParticleSpecies File\n");
	printf("------------------------\n\n");
	
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"r");
	fscanf(ft,"%d",&Nbdsp);
	printf("Nbdsp = %d\n",Nbdsp);
	for(int i = 0 ; i < Nbdsp ; i++){
		BodySpecie *pll = new  BodySpecie();
		bdsp.push_back(*pll);
		delete pll;
	}
	for(int i = 0 ; i < Nbdsp ; i++){
		bdsp[i].LoadFromFile(ft);
	}
	fclose(ft);
	/*
	 for(int i = 0 ; i < Nbdsp ; i++){
	 bdsp[i].affiche();
	 }
	 */
}


void ReadWrite::readOutContainer(char *directory, int & Npl, int & Nplr, int & Nco, int & Nelb, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb) noexcept {
	// Procedure which read the container file in Export directory at the begenning of the program
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sExport/container.txt",directory);
	printf("Uploading Container File\n");
	printf("------------------------\n\n");
	
	printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"r");
	// Read Data
	fscanf(ft,"%d",&Npl);
	fscanf(ft,"%d",&Nplr);
	fscanf(ft,"%d",&Nco);
	fscanf(ft,"%d",&Nelb);
	printf("Number of Plan = %d\n",Npl);
	printf("Number of Disk = %d\n",Nplr);
	printf("Number of Cone = %d\n",Nco);
	printf("Number of Elbow = %d\n\n",Nelb);
	// Upload Plan
	
	for(int i = 0 ; i < Npl ; i++){
		Plan *pll = new  Plan();
		pl.push_back(*pll);
		delete pll;
	}
	printf("push_back\n");
	for(int i = 0 ; i < Npl ; i++){
		pl[i].LoadFromFile(ft);
		pl[i].Numero(i);
	}
	// Upload PlanR
	for(int i = 0 ; i < Nplr ; i++){
		PlanR *plrl = new PlanR();
		plr.push_back(*plrl);
		delete plrl;
	}
	for(int i = 0 ; i < Nplr ; i++){
		plr[i].readFromFile(ft);
		plr[i].Numero(i);
	}
	// Upload Cone
	for(int i = 0 ; i < Nco ; i++){
		Cone *col = new Cone();
		co.push_back(*col);
		delete col;
	}
	for(int i = 0 ; i < Nco ; i++){
		co[i].readFromFile(ft);
		co[i].Numero(i);
	}
	// Upload Elbow
	for(int i = 0 ; i < Nelb ; i++){
		Elbow *elbl = new Elbow();
		elb.push_back(*elbl);
		delete elbl;
	}
	for(int i = 0 ; i < Nelb ; i++){
		elb[i].ReadFromFile(ft);
		elb[i].numero = i;
	}
	// Close File
	fclose(ft);
	fflush(ft);
	printf("Loading Container Finished\n");
	
	//printf("End readOutContainer\n");
}

void ReadWrite::readStart_stopContainer(char *directory, int & Npl, int & Nplr, int & Nco, int & Nelb, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb) noexcept {
	// Procedure which read the container file in Export directory at the begenning of the program
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sStart_stop/container.txt",directory);
	printf("Uploading Container File\n");
	printf("------------------------\n\n");
	
	printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"r");
	// Read Data
	fscanf(ft,"%d",&Npl);
	fscanf(ft,"%d",&Nplr);
	fscanf(ft,"%d",&Nco);
	fscanf(ft,"%d",&Nelb);
	printf("Number of Plan = %d\n",Npl);
	printf("Number of Disk = %d\n",Nplr);
	printf("Number of Cone = %d\n",Nco);
	printf("Number of Elbow = %d\n\n",Nelb);
	
	// Upload Plan
	for(int i = 0 ; i < Npl ; i++){
		Plan *pll = new  Plan();
		pl.push_back(*pll);
		delete pll;
	}
	
	for(int i = 0 ; i < Npl ; i++){
		pl[i].LoadFromFile(ft);
		pl[i].Numero(i);
	}
	// Upload PlanR
	for(int i = 0 ; i < Nplr ; i++){
		PlanR *plrl = new PlanR();
		plr.push_back(*plrl);
		delete plrl;
	}
	for(int i = 0 ; i < Nplr ; i++){
		plr[i].readFromFile(ft);
		plr[i].Numero(i);
	}
	// Upload Cone
	for(int i = 0 ; i < Nco ; i++){
		Cone *col = new Cone();
		co.push_back(*col);
		delete col;
	}
	for(int i = 0 ; i < Nco ; i++){
		co[i].readFromFile(ft);
		co[i].Numero(i);
	}
	// Upload Elbow
	for(int i = 0 ; i < Nelb ; i++){
		Elbow *elbl = new Elbow();
		elb.push_back(*elbl);
		delete elbl;
	}
	for(int i = 0 ; i < Nelb ; i++){
		elb[i].ReadFromFile(ft);
		elb[i].numero = i;
	}
	// Close File
	fclose(ft);
	fflush(ft);
	//printf("End readOutContainer\n");
}


void ReadWrite::writeStartStopContainer(char *directory, int & Npl, int & Nplr, int & Nco, int & Nelb, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb) noexcept {
	// Procedure which write in Start_stop directory the container file for eventually a restart of the progam
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sStart_stop/container.txt",directory);
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"w");
	// Write Data
	fprintf(ft,"%d\n",Npl);
	fprintf(ft,"%d\n",Nplr);
	fprintf(ft,"%d\n",Nco);
	fprintf(ft,"%d\n",Nelb);
	
	for(int i = 0 ; i < Npl ; i++){
		pl[i].WriteToFile(ft);
	}
	for(int i = 0 ; i < Nplr ; i++){
		plr[i].writeToFile(ft);
	}
	for(int i = 0 ; i < Nco ; i++){
		co[i].writeToFile(ft);
	}
	for(int i = 0 ; i < Nelb ; i++){
		elb[i].WriteToFile(ft);
	}
	// Close File
	fclose(ft);
	fflush(ft);
}

void ReadWrite::writeOutContainer(char *directory, int n,int & Npl, int & Nplr, int & Nco, int & Nelb, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb, int mode) noexcept {
	// Procedure which write in Out directory the container file for saving data
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	//printf("FileName : %s\n",fileName);
	// Open File
	if(mode == 0){
		sprintf(fileName,"%sOut/container_%d.txt",directory,n);
		ft = fopen(fileName,"w");
		
		fprintf(ft,"%d\n",Npl);
		fprintf(ft,"%d\n",Nplr);
		fprintf(ft,"%d\n",Nco);
		fprintf(ft,"%d\n",Nelb);
	}
	else{
		sprintf(fileName,"%sOut/container_%d.bin",directory,n);
		ft = fopen(fileName,"wb");
		
		fwrite(&Npl,sizeof(int),1,ft);
		fwrite(&Nplr,sizeof(int),1,ft);
		fwrite(&Nco,sizeof(int),1,ft);
		fwrite(&Nelb,sizeof(int),1,ft);
	}
	
	// Write Data
	for(int i = 0 ; i < Npl ; i++){
		pl[i].WriteOutFile(ft,mode);
	}
	for(int i = 0 ; i < Nplr ; i++){
		plr[i].writeOutFile(ft,mode);
	}
	for(int i = 0 ; i < Nco ; i++){
		co[i].writeOutFile(ft,mode);
	}
	for(int i = 0 ; i < Nelb ; i++){
		elb[i].WriteOutFile(ft,mode);
	}
	// Close File
	fclose(ft);
	fflush(ft);
}

void ReadWrite::readOutSphere(char *directory, int & Nsph, vector<Sphere> & sph, int limite) noexcept {
	char fileName[1024];
	FILE *ft;
	int b;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sExport/grain.txt",directory);
	printf("Uploading Sphere File\n");
	printf("---------------------\n\n");
	
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"r");
	if(limite == -9)
		fscanf(ft,"%d",&Nsph);
		else{
			fscanf(ft,"%d",&b);
			Nsph = limite;
		}
	printf("Number of Sphere = %d\n\n",Nsph);
	for(int i = 0 ; i < Nsph ; i++){
		Sphere *sphl = new Sphere();
		sphl->readFromFile(ft);
		sph.push_back(*sphl);
		delete sphl;
	}
	fclose(ft);
	fflush(ft);
}

void ReadWrite::readStart_stopSphere(char *directory, int & Nsph, vector<Sphere> & sph, int limite) noexcept {
	char fileName[1024];
	int b;
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sStart_stop/grain.txt",directory);
	printf("Uploading Sphere File\n");
	printf("---------------------\n\n");
	
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"r");
	if(limite == -9)
		fscanf(ft,"%d",&Nsph);
		else{
			fscanf(ft,"%d",&b);
			Nsph = limite;
		}
	
	printf("Number of Sphere = %d\n\n",Nsph);
	for(int i = 0 ; i < Nsph ; i++){
		Sphere *sphl = new Sphere();
		sphl->readStartStop(ft);
		sph.push_back(*sphl);
		delete sphl;
	}
	fclose(ft);
	fflush(ft);
}


void ReadWrite::writeStartStopSphere(char *directory, int & Nsph, vector<Sphere> & sph) noexcept {
	char fileName[1024];
	FILE *ft;
	int NsphNoBodies = Nsph;
	for(int i = 0 ; i < Nsph ; i++){
		NsphNoBodies -= sph[i].count();
	}
	
	// Making FileName
	sprintf(fileName,"%sStart_stop/grain.txt",directory);
	ft = fopen(fileName,"w");
	fprintf(ft,"%d\n",NsphNoBodies);
	for(int i = 0 ; i < Nsph ; i++){
		sph[i].writeToFile(ft);
	}
	fclose(ft);
	fflush(ft);
}

void ReadWrite::writeOutSphere(char *directory, int n, int & Nsph, vector<Sphere> & sph, int mode) noexcept {
	int Nl = Nsph;
	if(Nsph != 0){
		char fileName[1024];
		FILE *ft;
		
		for(int i = 0 ; i < Nsph ; i++){
			Nl -= sph[i].count();
		}
		
		if(mode == 0){
			// Making FileName
			sprintf(fileName,"%sOut/grain_%d.txt",directory,n);
			// Open File
			ft = fopen(fileName,"w");
			fprintf(ft,"%d\n",Nl);
		}
		else{
			// Making FileName
			sprintf(fileName,"%sOut/grain_%d.bin",directory,n);
			// Open File
			ft = fopen(fileName,"wb");
			fwrite(&Nl, sizeof(int), 1, ft);
		}
		for(int i = 0 ; i < Nsph ; i++){
			sph[i].writeOutFile(ft,i,mode);
		}
		fclose(ft);
		fflush(ft);
	}
}

void ReadWrite::readOutHollowBall(char *directory, int & Nhbl, vector<HollowBall> & hbl) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sExport/hollowBall.txt",directory);
	printf("Uploading HollowBall File\n");
	printf("-------------------------\n\n");
	
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"r");
	fscanf(ft,"%d",&Nhbl);
	
	for(int i = 0 ; i < Nhbl ; i++){
		HollowBall *sphl = new HollowBall();
		sphl->loadFromFile(ft);
		hbl.push_back(*sphl);
		delete sphl;
	}
	fclose(ft);
	fflush(ft);
	
}

void ReadWrite::writeStartStopHollowBall(char *directory, int & Nhbl, vector<HollowBall> & hbl) noexcept {
	char fileName[1024];
	FILE *ft;
	sprintf(fileName,"%sStart_stop/hollowBall.txt",directory);
	ft = fopen(fileName,"w");
	fprintf(ft,"%d\n",Nhbl);
	for(int i = 0 ; i < Nhbl ; i++)
		hbl[i].writeToFile(ft);
		fclose(ft);
		fflush(ft);
		
		}

void ReadWrite::writeOutHollowBall(char *directory, int n, int & Nhbl, vector<HollowBall> & hbl) noexcept {
	if(Nhbl != 0){
		char fileName[1024];
		FILE *ft;
		sprintf(fileName,"%sOut/hollowBall_%d.txt",directory,n);
		ft = fopen(fileName,"w");
		fprintf(ft,"%d\n",Nhbl);
		for(int i = 0 ; i < Nhbl ; i++)
			hbl[i].writeToFile(ft);
			fclose(ft);
			fflush(ft);
			}
}

void ReadWrite::readStart_stopHollowBall(char *directory, int & Nhbl, vector<HollowBall> & hbl) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sStart_stop/hollowBall.txt",directory);
	printf("Uploading HollowBall File\n");
	printf("-------------------------\n\n");
	
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"r");
	fscanf(ft,"%d",&Nhbl);
	
	printf("Nhbl = %d\n",Nhbl);
	
	for(int i = 0 ; i < Nhbl ; i++){
		HollowBall *sphl = new HollowBall();
		sphl->loadFromFile(ft);
		hbl.push_back(*sphl);
		delete sphl;
	}
	fclose(ft);
}


void ReadWrite::readOutBodies(char *directory, int & Nbd, vector<Body> & bd, int limite) noexcept {
	char fileName[1024];
	int b;
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sExport/particle.txt",directory);
	//printf("FileName : %s\n",fileName);
	// Open File
	printf("Uploading Body File\n");
	printf("-------------------\n\n");
	
	ft = fopen(fileName,"r");
	if(limite == -9)
		fscanf(ft,"%d",&Nbd);
		else{
			fscanf(ft,"%d",&b);
			Nbd = limite;
		}
	printf("Number of Body = %d\n\n",Nbd);
	for(int i = 0 ; i < Nbd ; i++){
		Body *bdl = new Body();
		bdl->LoadFromFile(ft);
		bd.push_back(*bdl);
		delete bdl;
	}
	fclose(ft);
	fflush(ft);
}

void ReadWrite::readStart_stopBodies(char *directory, int & Nbd, vector<Body> & bd, int limite) noexcept {
	char fileName[1024];
	int b;
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sStart_stop/particle.txt",directory);
	//printf("FileName : %s\n",fileName);
	// Open File
	printf("Uploading Body File\n");
	printf("-------------------\n\n");
	
	ft = fopen(fileName,"r");
	if(limite == -9)
		fscanf(ft,"%d",&Nbd);
		else{
			fscanf(ft,"%d",&b);
			Nbd = limite;
		}
	printf("Number of Body = %d\n\n",Nbd);
	for(int i = 0 ; i < Nbd ; i++){
		Body *bdl = new Body();
		bdl->ReadStartStopFile(ft);
		bd.push_back(*bdl);
		delete bdl;
	}
	fclose(ft);
	fflush(ft);
}



void ReadWrite::writeStartStopBodies(char *directory, int & Nbd, vector<Body> & bd,vector<Sphere> & sph) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sStart_stop/particle.txt",directory);
	//printf("FileName : %s\n",fileName);
	ft = fopen(fileName,"w");
	fprintf(ft,"%d\n",Nbd);
	for(int i = 0 ; i < Nbd ; i++){
		bd[i].WriteToFile(ft,sph);
	}
	fclose(ft);
	fflush(ft);
}

void ReadWrite::writeOutBodies(char *directory, int n, int & Nbd, vector<Body> & bd, int mode) noexcept {
	if(Nbd != 0){
		char fileName[1024];
		FILE *ft;
		if(mode == 0){
			// Making FileName
			sprintf(fileName,"%sOut/particle_%d.txt",directory,n);
			// Open File
			ft = fopen(fileName,"w");
			fprintf(ft,"%d\n",Nbd);
		}
		else{
			// Making FileName
			sprintf(fileName,"%sOut/particle_%d.bin",directory,n);
			// Open File
			ft = fopen(fileName,"wb");
			fwrite(&Nbd, sizeof(int), 1, ft);
		}
		for(int i = 0 ; i < Nbd ; i++){
			bd[i].WriteOutFile(ft,mode);
		}
		fclose(ft);
		fflush(ft);
	}
}

void ReadWrite::readOutData(char *directory, Gravity *gf, Data *dat) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sExport/data.txt",directory);
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"r");
	gf->LoadFromFile(ft);
	dat->LoadFromFile(ft);
	fclose(ft);
	fflush(ft);
}

void ReadWrite::readStartStopData(char *directory, Gravity *gf, Data *dat) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sStart_stop/data.txt",directory);
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"r");
	gf->LoadFromFile(ft);
	dat->LoadFromFile(ft);
	fclose(ft);
	fflush(ft);
}


void ReadWrite::writeOutData(char *directory, int n, Gravity *gf, Data *dat) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sOut/data_%d.txt",directory,n);
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"w");
	gf->WriteToFile(ft);
	dat->WriteToFile(ft);
	fclose(ft);
	fflush(ft);
	/*
	 char commande[1024];
	 sprintf(commande,"gzip -f %s",fileName);
	 system(commande);
	 */
}

void ReadWrite::writeStartStopData(char *directory, Gravity *gf, Data *dat) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sStart_stop/data.txt",directory);
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"w");
	gf->WriteToFile(ft);
	dat->WriteToFile(ft);
	fclose(ft);
	fflush(ft);
}


void ReadWrite::writeOutContact(char *directory, int n, int & Nct, Contact *ct, Data & dat) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sOut/contact_%d.txt",directory,n);
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"w");
	//printf("Nct = %d\n",Nct);
	fprintf(ft,"%d\n",Nct);
	for(int i = 0 ; i < Nct ; i++){
		ct[i].WriteOutFile(ft,dat.TIME);
		//printf("Contact %d written\n",i);
	}
	//printf("All contact are written");
	fclose(ft);
	fflush(ft);
	//printf("File %s has been closed\n",fileName);
}

void ReadWrite::writeOutContactDetails(char *directory, int n, int & Nct, Contact *ct, Data & dat) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sOut/contactD_%d.txt",directory,n);
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"w");
	//printf("Nct = %d\n",Nct);
	fprintf(ft,"%d\n",Nct);
	for(int i = 0 ; i < Nct ; i++){
		ct[i].WriteOutFileDetails(ft,dat.TIME);
		//printf("Contact %d written\n",i);
	}
	//printf("All contact are written");
	fclose(ft);
	fflush(ft);
	//printf("File %s has been closed\n",fileName);
}
