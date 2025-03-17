#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "../../Includes/Object/BodySpecie.h"

BodySpecie::BodySpecie() noexcept {
	Ng = 0;
	m = 0;
	Ine_1[0][0] = 0;
	Ine_1[0][1] = 0;
	Ine_1[0][2] = 0;
	Ine_1[1][0] = 0;
	Ine_1[1][1] = 0;
	Ine_1[1][2] = 0;
	Ine_1[2][0] = 0;
	Ine_1[2][1] = 0;
	Ine_1[2][2] = 0;
	sp = 0;
    FeretMax = 0;
}

BodySpecie::~BodySpecie() noexcept {}

void BodySpecie::FreeMemory() noexcept {
	num.clear();
	xl.clear();
	yl.clear();
	zl.clear();
	rl.clear();
}

void BodySpecie::LoadFromFile(FILE *ft) noexcept {
	double x, y, z, r;
    if(fscanf(ft,"%d",&Ng) != 1)
        return;
	FreeMemory();
	for(int j = 0 ; j < Ng ; j++){
		num.push_back(j);
		fscanf(ft,"%lf\t%lf\t%lf\t%lf\n",&x,&y,&z,&r);
		xl.push_back(x);
		yl.push_back(y);
		zl.push_back(z);
		rl.push_back(r);
	}
	fscanf(ft,"%lf\t%lf",&m,&FeretMax);
	fscanf(ft,"%lf\t%lf\t%lf\n",&Ine_1[0][0],&Ine_1[0][1],&Ine_1[0][2]);
	fscanf(ft,"%lf\t%lf\t%lf\n",&Ine_1[1][0],&Ine_1[1][1],&Ine_1[1][2]);
	fscanf(ft,"%lf\t%lf\t%lf\n",&Ine_1[2][0],&Ine_1[2][1],&Ine_1[2][2]);	
}

void BodySpecie::Display() const noexcept {
    printf("%d\n",Ng);
    for(int j = 0 ; j < Ng ; j++){
        printf("\t%e\t%e\t%e\t%e\n",xl[j],yl[j],zl[j],rl[j]);
    }
    printf("%e\t%e\n",m,FeretMax);
    printf("%e\t%e\t%e\n",Ine_1[0][0],Ine_1[0][1],Ine_1[0][2]);
    printf("%e\t%e\t%e\n",Ine_1[1][0],Ine_1[1][1],Ine_1[1][2]);
    printf("%e\t%e\t%e\n",Ine_1[2][0],Ine_1[2][1],Ine_1[2][2]);
    printf("\n");
}
