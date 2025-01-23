#include "../Includes/Option.h"
#include <cstring>
#include <iostream>
#include <cmath>
#include <sys/stat.h>
#include <filesystem>


Option::Option() noexcept {
	NoRotation = 0;
	Vrdm = 0.0;
	Wrdm = 0.0;
	fps = -1;
	stop_time = -1.;
	start_time = -1.;
	tune_mu = -1;
	tune_e = -1;
	cancelG = false;
	restart = 0;
	limitNg = -9;
	limitNbd = -9;
	cancelVel = 0;
	RdmVel = 0;
	mode = 0;
	parallel = 0;
	CoeffAffinity = 0;
	Nprocess = 1;
	compression = 0;
	sprintf(directory,"./");
	Nlpl = 0;
	Nlplr = 0;
	Nlco = 0;
	force_npl = -1;
	Npl_nn = -9;
	angle_nn = M_PI_4;
}

int Option::Management(char **argv, int argc) noexcept {
	list = argv;
	Nlist = argc;
	for(int i = 1 ; i < argc ; i++){
		if(argv[i][0] == 45){
			switch(argv[i][1]){
				case 'g':
					if(!strcmp(argv[i], "-gvd")){
						sscanf(argv[i+1],"%lf",&visco);
						i++;
					}
					if(!strcmp(argv[i], "-globalviscousdamping")){
						sscanf(argv[i+1],"%lf",&visco);
						i++;
					}
					break;
				case 'v':
					if(!strcmp(argv[i], "-version")){
						printf("SanPanda Version 5.0 c++ (by F. Ludewig)\n");
						return 0;
					}
					if(!strcmp(argv[i], "-v")){
						printf("SanPanda Version 5.0 c++ (by F. Ludewig)\n");
						return 0;
					}
					if(!strcmp(argv[i], "-velocitycancel")){
						printf("Cancel velocity\n");
						cancelVel = 1;
					}
					if(!strcmp(argv[i], "-vc")){
						printf("Cancel velocity\n");
						cancelVel = 1;
					}
					if(!strcmp(argv[i], "-velocityrandom")){
						printf("Random velocity\n");
						sscanf(argv[i+1],"%lf",&Vrdm);
						i++;
						sscanf(argv[i+1],"%lf",&Wrdm);
						i++;
						RdmVel = 1;
					}
					if(!strcmp(argv[i], "-vrdm")){
						printf("Random velocity\n");
						sscanf(argv[i+1],"%lf",&Vrdm);
						i++;
						sscanf(argv[i+1],"%lf",&Wrdm);
						i++;
						RdmVel = 1;
					}
					break;
				case 'h':
					if(!strcmp(argv[i], "-help")){
						printf("SanPanda Version 5.0 c++\nOptions are :\n");
						printf("\t-p or -parallel for parallel contact detection with OpenMP\n");
						printf("\t-s or -sequential for sequential contact detection (default option)\n");
						printf("\t-d or -directory to indicate the current directory of the simulation (default option is the current directory)\n");
						printf("\t-a or -affinitycache for active affinity cache memory followed by a tag (integer)\n\t\t\texample : -a 1\n");
						printf("\t-mc or modecompaction for compaction mode followed by compaction data : Numbers of first and last tap, Reduce Acceleration, Frequency\n\t\t\texample : -c 0 1000 4 25\n");
						printf("\t-e or -evolution for simple evolution of the original data (default option) in opposition to compaction option\n\n");
						printf("\t-mpq or -modepowderpaq for compaction mode with PowderPaQ way, followed by compaction data : Numbers of first and last tap, freefall height and upward velocity\n\t\t\texample : -mpq 0 1000 0.01 0.001\n\n");
						printf("\t-tgz to archive of all directories after simulation\n\n");
						printf("\t-nr or -norotation to lock all dergrees of freedom of rotation\n\n");
						printf("\t-r or -restart for restart simulation at the last saved state\n");
						printf("\t-N for cut off the number of spheres and particles (works only without GrainBorders)\n");
						printf("\t\tThe wanted number of spheres\n");
						printf("\t\tThe wanted number of particles\n\n");
						printf("\t-oc or -outcontact in order to have the contact files in the Out directory\n");
						printf("\t-ocv or -outcontactv in order to have the contact files in the Out directory used in SandPandaViewer\n");
						printf("\t-gvd or -globalviscousdamping for activating a viscous force on spherical particles.\n");
						printf("\t\tThe value of the viscous damping coefficient\n");
						return 0;
					}
					if(!strcmp(argv[i], "-h")){
						printf("SanPanda Version 5.0 c++\nOptions are :\n");
						printf("\t-p or -parallel for parallel contact detection with OpenMP\n");
						printf("\t-s or -sequential for sequential contact detection (default option)\n");
						printf("\t-d or -directory to indicate the current directory of the simulation (default option is the current directory)\n");
						printf("\t-a or -affinitycache for active affinity cache memory followed by a tag (integer)\n\t\t\texample : -a 1\n");
						printf("\t-mc or modecompaction for compaction mode followed by compaction data : Numbers of first and last tap, Reduce Acceleration, Frequency\n\t\t\texample : -c 0 1000 4 25\n");
						printf("\t-e or -evolution for simple evolution of the original data (default option) in opposition to compaction option\n\n");
						printf("\t-mpq or -modepowderpaq for compaction mode with PowderPaQ way, followed by compaction data : Numbers of first and last tap, freefall height and upward velocity\n\t\t\texample : -mpq 0 1000 0.01 0.001\n\n");
						printf("\t-tgz to archive of all directories after simulation\n\n");
						printf("\t-nr or -norotation to lock all dergrees of freedom of rotation\n\n");
						printf("\t-r or -restart for restart simulation at the last saved state\n");
						printf("\t-N for cut off the number of spheres and particles (works only without GrainBorders)\n");
						printf("\t\tThe wanted number of spheres\n");
						printf("\t\tThe wanted number of particles\n\n");
						printf("\t-oc or -outcontact in order to have the contact files in the Out directory\n");
						printf("\t-ocv or -outcontactv in order to have the contact files in the Out directory used in SandPandaViewer\n");
						printf("\t-gvd or -globalviscousdamping for activating a viscous force on spherical particles.\n");
						printf("\t\tThe value of the viscous damping coefficient\n");
						return 0;
					}
					break;
				case 'p':
					if(!strcmp(argv[i], "-parallel")){
						printf("Parallel\n");
						sscanf(argv[i+1],"%d",&Nprocess);
						i++;
						parallel = 1;
					}
					if(!strcmp(argv[i], "-p")){
						printf("Parallel\n");
						sscanf(argv[i+1],"%d",&Nprocess);
						i++;
						parallel = 1;
					}
					break;
				case 's':
					if(!strcmp(argv[i], "-sequential")){
						printf("Sequential\n");
						parallel = 0;
					}
					if(!strcmp(argv[i], "-s")){
						printf("Sequential\n");
						parallel = 0;
					}
					break;
				case 'd':
					if(!strcmp(argv[i], "-directory")){
						sscanf(argv[i+1],"%s",directory);
						printf("Directory : %s\n",directory);
						i++;
					}
					if(!strcmp(argv[i], "-d")){
						sscanf(argv[i+1],"%s",directory);
						printf("Directory : %s\n",directory);
						i++;
					}
					break;
				case 'a':
					if(!strcmp(argv[i], "-affinitycache")){
						sscanf(argv[i+1],"%d",&CoeffAffinity);
						printf("Activation Affinity Cache Memory for tag = %d\n",CoeffAffinity);
						i++;
					}
					if(!strcmp(argv[i], "-a")){
						sscanf(argv[i+1],"%d",&CoeffAffinity);
						printf("Activation Affinity Cache Memory for tag = %d\n",CoeffAffinity);
						i++;
					}
					break;
				case 'e':
					sscanf(argv[i+1],"%lf",&tune_e);
					i+=1;
					break;
				case 'c':
					if(!strcmp(argv[i], "-cg"))
						cancelG = true;
					break;
				case 'm':
					if(!strcmp(argv[i], "-mu")){
						sscanf(argv[i+1],"%lf",&tune_mu);
						i+=1;
					}
					if(!strcmp(argv[i], "-modecompaction")){
						sscanf(argv[i+1],"%d",&NtapMin);
						sscanf(argv[i+2],"%d",&NtapMax);
						sscanf(argv[i+3],"%lf",&Gamma);
						sscanf(argv[i+4],"%lf",&Freq);
						printf("Mode Compaction : tap from %d to %d\n",NtapMin,NtapMax);
						printf("Tap characteristic : Gamma = %e, f = %e and A = %e\n",Gamma,Freq,Gamma*9.81/(4*M_PI*M_PI*Freq*Freq));
						i+=4;
						mode = 1;
					}
					if(!strcmp(argv[i], "-mc")){
						sscanf(argv[i+1],"%d",&NtapMin);
						sscanf(argv[i+2],"%d",&NtapMax);
						sscanf(argv[i+3],"%lf",&Gamma);
						sscanf(argv[i+4],"%lf",&Freq);
						printf("Mode Compaction : tap from %d to %d\n",NtapMin,NtapMax);
						printf("Tap characteristic : Gamma = %e, f = %e and A = %e\n",Gamma,Freq,Gamma*9.81/(4*M_PI*M_PI*Freq*Freq));
						i+=4;
						mode = 1;
					}
					
					if(!strcmp(argv[i], "-modeevolution")){
						printf("Mode Evolution\n");
						mode = 0;
					}
					if(!strcmp(argv[i], "-me")){
						printf("Mode Evolution\n");
						mode = 0;
					}
					if(!strcmp(argv[i], "-modepowderpaq")){
						mode = 2;
						sscanf(argv[i+1],"%d",&NtapMin);
						sscanf(argv[i+2],"%d",&NtapMax);
						sscanf(argv[i+3],"%lf",&PQheight);
						sscanf(argv[i+4],"%lf",&PQVel);
						printf("Mode PowderPaQ : tap from %d to %d with H = %e and V = %e\n",NtapMin,NtapMax,PQheight,PQVel);
						i+=4;
					}
					if(!strcmp(argv[i], "-mpq")){
						mode = 2;
						sscanf(argv[i+1],"%d",&NtapMin);
						sscanf(argv[i+2],"%d",&NtapMax);
						sscanf(argv[i+3],"%lf",&PQheight);
						sscanf(argv[i+4],"%lf",&PQVel);
						printf("Mode PowderPaQ : tap from %d to %d with H = %e and V = %e\n",NtapMin,NtapMax,PQheight,PQVel);
						i+=4;
					}
					if(!strcmp(argv[i], "-melt")){
						mode = 3;
						sscanf(argv[i+1],"%lf",&vr);
						i++;
						sscanf(argv[i+1],"%lf",&delayVr);
						i++;
						printf("Mode Melting dr = %e (vr = %e)in %e [s]\n",vr,vr/delayVr,delayVr);
					}
					break;
				case 't':
					if(!strcmp(argv[i], "-tgz")){
						compression = 1;
					}
					if(!strcmp(argv[i], "-t0")){
						sscanf(argv[i+1],"%lf",&start_time);
						i++;
					}
					if(!strcmp(argv[i], "-t1")){
						sscanf(argv[i+1],"%lf",&stop_time);
						i++;
					}
					
					
					break;
				case 'n':
					if(!strcmp(argv[i], "-norotation")){
						NoRotation = 1;
					}
					if(!strcmp(argv[i], "-nr")){
						NoRotation = 1;
					}
					if(!strcmp(argv[i], "-normalnoise")){
						sscanf(argv[i+1],"%d",&Npl_nn);
						i++;
						sscanf(argv[i+1],"%lf",&angle_nn);
						i++;
					}
					if(!strcmp(argv[i], "-nn")){
						sscanf(argv[i+1],"%d",&Npl_nn);
						i++;
						sscanf(argv[i+1],"%lf",&angle_nn);
						i++;
					}
					break;
				case 'r':
					if(!strcmp(argv[i], "-restart")){
						restart = 1;
					}
					if(!strcmp(argv[i], "-r")){
						restart = 1;
					}
					break;
				case 'o':
					if(!strcmp(argv[i], "-ocv")){
						outContact += 1;
					}
					if(!strcmp(argv[i], "-outcontactv")){
						outContact += 1;
					}
					if(!strcmp(argv[i], "-oc")){
						outContact += 2;
					}
					if(!strcmp(argv[i], "-outcontact")){
						outContact += 2;
					}
					break;
				case 'N':
					sscanf(argv[i+1],"%d",&limitNg);
					sscanf(argv[i+2],"%d",&limitNbd);
					i+=2;
					printf("Limit Ng to %d\n",limitNg);
					printf("Limit Nbd to %d\n",limitNbd);
					break;
				case 'f':
					if(!strcmp(argv[i], "-fps")){
						sscanf(argv[i+1],"%d",&fps);
						i++;
					}
					if(!strcmp(argv[i], "-fusion")){
						// Plan
						sscanf(argv[i+1],"%d",&Nlpl);
						i++;
						for(int m = 0 ; m < Nlpl ; m++){
							sscanf(argv[i+1],"%d",&listplan[m]);
							i++;
						}
						// PlanR
						sscanf(argv[i+1],"%d",&Nlplr);
						i++;
						for(int m = 0 ; m < Nlplr ; m++){
							sscanf(argv[i+1],"%d",&listplanR[m]);
							i++;
						}
						// Cone
						sscanf(argv[i+1],"%d",&Nlco);
						i++;
						for(int m = 0 ; m < Nlco ; m++){
							sscanf(argv[i+1],"%d",&listcone[m]);
							i++;
						}
					}
					break;
			}
		}
	}
	return 1;
}

int Option::DirectoryManagement() noexcept {
	// Controle de l existance des repertoires
	char commande[2048];
	
	// Test de l existance du repertoire Export
	sprintf(commande,"%sExport",directory);
    if(!std::filesystem::exists(commande)){
		// Si non : creation
		printf("The %s directory doesn't exists\n",commande);
		return 0;
	}

	
	if(restart == 1){
		// Test de l existance du repertoire Start_stop Si l'option restart est activee
		sprintf(commande,"%s/Start_stop",directory);
		// Test de l existance du repertoire
		if(!std::filesystem::exists(commande)){
			// Si non : creation
			printf("The current directory doesn't exists\n");
			return 0;
		}
		// Test de l existance du repertoire Start_stop Si l'option restart est activee
		sprintf(commande,"%s/Out",directory);
		// Test de l existance du repertoire
		if(!std::filesystem::exists(commande)){
			// Si non : creation
            std::filesystem::create_directory(commande);
		}
	}
	else{
		// Test de l existance du repertoire Start_stop Si l'option restart est activee
		sprintf(commande,"%s/Start_stop",directory);
		// Test de l existance du repertoire
		struct stat st;
		if(!std::filesystem::exists(commande)){
			// Si non : creation
            std::filesystem::create_directory(commande);
        }
		else{
			// Si oui : creation
            //std::filesystem::remove_all(commande);
            //std::filesystem::create_directory(commande);
        }
		// Test de l existance du repertoire Start_stop Si l'option restart est activee
		sprintf(commande,"%s/Out",directory);
		// Test de l existance du repertoire
		if(!std::filesystem::exists(commande)){
			// Si non : creation
            std::filesystem::create_directory(commande);
        }
		else{
			// Si non : creation
            //std::filesystem::remove_all(commande);
            //std::filesystem::create_directory(commande);
        }
	}
	return 1;
}

void Option::Record() noexcept {
	// Enregistrement des options
	FILE *ft;
	char FileName[1024];
	sprintf(FileName,"%s/option.txt",directory);
	ft =fopen(FileName,"w");
	for(int i = 1 ; i < Nlist ; i++){
		if(list[i][0] == 45){
			fprintf(ft,"\n%s ",list[i]);
		}
		else{
			fprintf(ft,"%s ",list[i]);
		}
	}
	fclose(ft);
	fflush(ft);
}

void Option::InData(Data & dat, Gravity & gf, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co) noexcept {
	// Fabrication du MasterSolid memorise dans dat
	dat.mas = new MasterSolid();
	// Plan
	dat.mas->initPlan(Nlpl);
	for(int m = 0 ; m < Nlpl ; m++){
		dat.mas->addPlan(pl,m,listplan[m]);
	}
	// PlanR
	dat.mas->initPlanR(Nlplr);
	for(int m = 0 ; m < Nlplr ; m++){
		dat.mas->addPlanR(plr,m,listplanR[m]);
	}
	// Cone
	dat.mas->initCone(Nlco);
	for(int m = 0 ; m < Nlco ; m++){
		dat.mas->addCone(co,m,listcone[m]);
	}
	
	dat.mas->ComputePara();
	
	if(tune_mu != -1){
		dat.mu = tune_mu;
		dat.muD = tune_mu;
		dat.muS = tune_mu;
	}
	
	if(cancelG)
		gf.G = 0.0;
	if(tune_e != -1)
		dat.en = tune_e;
	

	if(fps > 0){
		dat.dts = 1./fps;
	}
	
	if(start_time != -1){
		printf("Start_time\n");
		dat.TIME = start_time;
	}
	
	if(stop_time != -1){
		dat.Total = stop_time;
	}
	
	dat.outContact = outContact;
	
	
	if(Npl_nn == -1){
		for(int i = 0 ; i < pl.size() ; i++)
			pl[i].SetAlpha(angle_nn/180*M_PI);
	}
	else{
		if(Npl_nn != -9)
			pl[Npl_nn].SetAlpha(angle_nn/180*M_PI);
	}
}
