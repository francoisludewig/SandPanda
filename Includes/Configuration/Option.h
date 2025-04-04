#pragma once

#include <stdio.h>
#include "Data.h"
#include "Gravity.h"

class Option{
private:
	char **list;
	int Nlist;
public:
	double visco,Vrdm,Wrdm,tune_e,tune_mu,Gamma,Freq,PQheight,PQVel,vr,delayVr,start_time,stop_time;
	int cancelVel,RdmVel,Nprocess,parallel,CoeffAffinity,NtapMin,NtapMax,mode,compression,NoRotation,restart,outContact,limitNg,limitNbd,fps;
	char directory[1024];
	bool cancelG;
	
	int listplan[10],Nlpl;
	int listplanR[10],Nlplr;
	int listcone[10],Nlco;
	
	int force_npl;
	double min,max,delay;
	char axe;
	
	int Npl_nn;
	double angle_nn;

    bool isMonitoringActivated {false};
    char processName[32];
	char scriptPath[2048];

	Option() noexcept;
	int Management(char **argv, int argc) noexcept;
	int DirectoryManagement() noexcept;
	void Record() noexcept;
	void InData(Data & dat, Gravity &gf, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co) noexcept;
};
