#include "../../Includes/Repository/ReadWrite.h"

#include "../../Includes/Solids/Velocity.h"
#include "../../Includes/Configuration/Gravity.h"
#include "../../Includes/Configuration/Configuration.h"
#include "../../Includes/Solids/Plan.h"
#include "../../Includes/Solids/PlanR.h"
#include "../../Includes/Solids/Cone.h"
#include "../../Includes/Solids/Elbow.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/Body.h"
#include "../../Includes/Solids/BodySpecie.h"
#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Solids/HollowBall.h"

void ReadWrite::readOutBodySpecie(char *directory,vector<BodySpecie> & bdsp) noexcept {
	char fileName[1024];
	int Nbdsp = 0;
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


void ReadWrite::readOutContainer(char *directory, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb) noexcept {
	// Procedure which read the container file in Export directory at the begenning of the program
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sExport/container.txt",directory);
	printf("Uploading Container File\n");
	printf("------------------------\n\n");

	printf("FileName : %s\n",fileName);

	int Npl = 0, Nplr = 0, Nco = 0, Nelb = 0;

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


	pl = std::vector<Plan>(Npl);
	plr = std::vector<PlanR>(Nplr);
	co = std::vector<Cone>(Nco);
	elb = std::vector<Elbow>(Nelb);
	printf("push_back\n");
	for(int i = 0 ; i < Npl ; i++){
		pl[i].LoadFromFile(ft);
		pl[i].Numero(i);
	}

	for(int i = 0 ; i < Nplr ; i++){
		plr[i].readFromFile(ft);
		plr[i].Numero(i);
	}

	for(int i = 0 ; i < Nco ; i++){
		co[i].readFromFile(ft);
		co[i].Numero(i);
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

void ReadWrite::readStart_stopContainer(char *directory, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb) noexcept {
	// Procedure which read the container file in Export directory at the begenning of the program
	char fileName[1024];
	int Npl = 0;
	int Nplr = 0;
	int Nco = 0;
	int Nelb = 0;
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


	pl = std::vector<Plan>(Npl);
	plr = std::vector<PlanR>(Nplr);
	co = std::vector<Cone>(Nco);
	elb = std::vector<Elbow>(Nelb);


	// Upload Plan
	for(int i = 0 ; i < pl.size() ; i++) {
		pl[i].LoadFromFile(ft);
		pl[i].Numero(i);
	}
	// Upload PlanR
	for(int i = 0 ; i < plr.size() ; i++) {
		plr[i].readFromFile(ft);
		plr[i].Numero(i);
	}
	// Upload Cone
	for(int i = 0 ; i < co.size() ; i++){
		co[i].readFromFile(ft);
		co[i].Numero(i);
	}
	// Upload Elbow
	for(int i = 0 ; i < elb.size() ; i++){
		elb[i].ReadFromFile(ft);
		elb[i].numero = i;
	}
	// Close File
	fclose(ft);
	fflush(ft);
	//printf("End readOutContainer\n");
}


void ReadWrite::writeStartStopContainer(char *directory, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb) noexcept {
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
	fprintf(ft,"%d\n",static_cast<int>(pl.size()));
	fprintf(ft,"%d\n",static_cast<int>(plr.size()));
	fprintf(ft,"%d\n",static_cast<int>(co.size()));
	fprintf(ft,"%d\n",static_cast<int>(elb.size()));

	for(const auto& plan : pl)
		plan.WriteToFile(ft);

	for(const auto& disk : plr)
		disk.writeToFile(ft);

	for(const auto& cone : co)
		cone.writeToFile(ft);

	for(const auto& elbow : elb)
		elbow.WriteToFile(ft);

	// Close File
	fclose(ft);
	fflush(ft);
}

void ReadWrite::writeOutContainer(char *directory, int n, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb, int mode) noexcept {
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

		fprintf(ft,"%d\n",static_cast<int>(pl.size()));
		fprintf(ft,"%d\n",static_cast<int>(plr.size()));
		fprintf(ft,"%d\n",static_cast<int>(co.size()));
		fprintf(ft,"%d\n",static_cast<int>(elb.size()));
	}

	// Write Data
	for(const auto& plan : pl)
		plan.WriteOutFile(ft,mode);

	for(const auto& disk : plr)
		disk.writeOutFile(ft,mode);

	for(const auto& cone : co){
		cone.writeOutFile(ft,mode);
	}
	for(const auto& elbow : elb)
		elbow.WriteOutFile(ft,mode);

	// Close File
	fclose(ft);
	fflush(ft);
}

void ReadWrite::readOutSphere(char *directory, vector<Sphere> & sph, int limite) noexcept {
	char fileName[1024];
	int Nsph = 0;
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
		Sphere sphl;
		sphl.readFromFile(ft);
		sph.push_back(std::move(sphl));
	}
	fclose(ft);
	fflush(ft);
}

void ReadWrite::readStart_stopSphere(char *directory, vector<Sphere> & sph, int limite) noexcept {
	char fileName[1024];
	int b;
	int Nsph = 0;
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
	sph.resize(Nsph);
	for(int i = 0 ; i < sph.size() ; i++)
		sph[i].readStartStop(ft);

	fclose(ft);
	fflush(ft);
}


void ReadWrite::writeStartStopSphere(char *directory, vector<Sphere> & sph) noexcept {
	char fileName[1024];
	FILE *ft;
	int NsphNoBodies = static_cast<int>(sph.size());
	for(int i = 0 ; i < sph.size() ; i++){
		NsphNoBodies -= sph[i].count();
	}

	// Making FileName
	sprintf(fileName,"%sStart_stop/grain.txt",directory);
	ft = fopen(fileName,"w");
	fprintf(ft,"%d\n",NsphNoBodies);
	for(int i = 0 ; i < sph.size() ; i++){
		sph[i].writeToFile(ft);
	}
	fclose(ft);
	fflush(ft);
}

void ReadWrite::writeOutSphere(char *directory, int n, vector<Sphere> & sph, int mode) noexcept {
	int Nl = sph.size();
	if(!sph.empty()){
		char fileName[1024];
		FILE *ft;

		for(int i = 0 ; i < sph.size() ; i++){
			if(sph[i].IsIntergationDisabled())
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
		for(int i = 0 ; i < sph.size() ; i++){
			if(!sph[i].IsIntergationDisabled())
				sph[i].writeOutFile(ft,i,mode);
		}
		fclose(ft);
		fflush(ft);
	}
}

void ReadWrite::readOutHollowBall(char *directory, vector<HollowBall> & hbl) noexcept {
	char fileName[1024];
	int Nhbl = 0;
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

void ReadWrite::writeStartStopHollowBall(char *directory, vector<HollowBall> & hbl) noexcept {
	char fileName[1024];
	FILE *ft;
	sprintf(fileName,"%sStart_stop/hollowBall.txt",directory);
	ft = fopen(fileName,"w");
	fprintf(ft,"%d\n",static_cast<int>(hbl.size()));
	for(int i = 0 ; i < hbl.size() ; i++)
		hbl[i].writeToFile(ft);
	fclose(ft);
	fflush(ft);

}

void ReadWrite::writeOutHollowBall(char *directory, int n, vector<HollowBall> & hbl) noexcept {
	if(!hbl.empty()){
		char fileName[1024];
		FILE *ft;
		sprintf(fileName,"%sOut/hollowBall_%d.txt",directory,n);
		ft = fopen(fileName,"w");
		fprintf(ft,"%d\n",static_cast<int>(hbl.size()));
		for(const auto& hollowBall : hbl)
			hollowBall.writeToFile(ft);
		fclose(ft);
		fflush(ft);
	}
}

void ReadWrite::readStart_stopHollowBall(char *directory, vector<HollowBall> & hbl) noexcept {
	char fileName[1024];
	int Nhbl = 0;
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


void ReadWrite::readOutBodies(char *directory, vector<Body> & bd, int limite) noexcept {
	char fileName[1024];
	int b;
	int Nbd = 0;
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

	bd = std::vector<Body>(Nbd);

	printf("Number of Body = %d\n\n",Nbd);
	for(int i = 0 ; i < Nbd ; i++)
		bd[i].LoadFromFile(ft);

	fclose(ft);
	fflush(ft);
}

void ReadWrite::readStart_stopBodies(char *directory, vector<Body> & bd, int limite) noexcept {
	char fileName[1024];
	int b;
	int Nbd = 0;
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
	bd = std::vector<Body>(Nbd);
	for(int i = 0 ; i < bd.size() ; i++)
		bd[i].ReadStartStopFile(ft);

	fclose(ft);
	fflush(ft);
}



void ReadWrite::writeStartStopBodies(char *directory, vector<Body> & bd,vector<Sphere> & sph) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sStart_stop/particle.txt",directory);
	//printf("FileName : %s\n",fileName);
	ft = fopen(fileName,"w");
	fprintf(ft,"%d\n",static_cast<int>(bd.size()));
	for(int i = 0 ; i < bd.size() ; i++){
		bd[i].WriteToFile(ft,sph);
	}
	fclose(ft);
	fflush(ft);
}

void ReadWrite::writeOutBodies(char *directory, int n, vector<Body> & bd, int mode) noexcept {
	if(!bd.empty()){
		char fileName[1024];
		FILE *ft;
		if(mode == 0){
			// Making FileName
			sprintf(fileName,"%sOut/particle_%d.txt",directory,n);
			// Open File
			ft = fopen(fileName,"w");
			fprintf(ft,"%d\n",static_cast<int>(bd.size()));
		}

		for(const auto& body : bd){
			body.WriteOutFile(ft,mode);
		}
		fclose(ft);
		fflush(ft);
	}
}

void ReadWrite::readOutData(char *directory, Gravity& gf, Configuration& dat) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sExport/data.txt",directory);
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"r");
	gf.LoadFromFile(ft);
	dat.LoadFromFile(ft);
	fclose(ft);
	fflush(ft);
}

void ReadWrite::readStartStopData(char *directory, Gravity& gf, Configuration& dat) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sStart_stop/data.txt",directory);
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"r");
	gf.LoadFromFile(ft);
	dat.LoadFromFile(ft);
	fclose(ft);
	fflush(ft);
}


void ReadWrite::writeOutData(char *directory, int n, Gravity& gf, Configuration& dat) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sOut/data_%d.txt",directory,n);
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"w");
	gf.WriteToFile(ft);
	dat.WriteToFile(ft);
	fclose(ft);
	fflush(ft);
	/*
	 char commande[1024];
	 sprintf(commande,"gzip -f %s",fileName);
	 system(commande);
	 */
}

void ReadWrite::writeStartStopData(char *directory, Gravity& gf, Configuration& dat) noexcept {
	char fileName[1024];
	FILE *ft;
	// Making FileName
	//printf("Directory : %s\n",directory);
	sprintf(fileName,"%sStart_stop/data.txt",directory);
	//printf("FileName : %s\n",fileName);
	// Open File
	ft = fopen(fileName,"w");
	gf.WriteToFile(ft);
	dat.WriteToFile(ft);
	fclose(ft);
	fflush(ft);
}


void ReadWrite::writeOutContact(char *directory, int n, int Nct, Contact *ct, Configuration & dat) noexcept {
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

void ReadWrite::writeOutContactDetails(char *directory, int n, int & Nct, Contact *ct, Configuration & dat) noexcept {
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
