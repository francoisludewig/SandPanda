#include "../../Includes/Solids/Cone.h"
#include "../../Includes/Solids/PlanR.h"
#include <vector>

Cone::Cone() noexcept :Solid(),
	r0(0.25), r1(0.25), h(0.5), dr(0), periodic(-9), in(0),
    numTop(-9), numBottom(-9), lxTop(0), lyTop(0), lzTop(0),
    lxBottom(0),lyBottom(0),lzBottom(0), top(nullptr), bottom(nullptr) {}

void Cone::readFromFile(FILE *ft) noexcept{
	Solid::LoadFromFile(ft);
	fscanf(ft,"%lf\t%lf\t%lf\t%lf\n",&h,&r0,&r1,&dr);
	fscanf(ft,"%d\t%d\t%d\n",&in,&numTop,&numBottom);
	Solid::LoadAccelerationFromFile(ft);
}

void Cone::LimitLink(std::vector<PlanR> & plr) noexcept {
    if(numTop != -9){
        top = &plr[numTop];
        top->SetForce(1);
        lxTop = top->X() - x;
        lyTop = top->Y() - y;
        lzTop = top->Z() - z;
    }
    if(numBottom != -9){
        bottom = &plr[numBottom];
        bottom->SetForce(1);
        lxBottom = bottom->X() - x;
        lyBottom = bottom->Y() - y;
        lzBottom = bottom->Z() - z;
    }
}


void Cone::LimitForce() noexcept {
    if(top != nullptr){
        Fx += top->GetFx();
        Fy += top->GetFy();
        Fz += top->GetFz();
    }
    if(bottom != nullptr){
        Fx += bottom->GetFx();
        Fy += bottom->GetFy();
        Fz += bottom->GetFz();
    }
}

void Cone::LimitUpdate() noexcept {
    if(top != nullptr){
        top->X(x + lxTop);
        top->Y(y + lyTop);
        top->Z(z + lzTop);
        top->Vx(vx);
        top->Vy(vy);
        top->Vz(vz);
    }
    if(bottom != nullptr){
        bottom->X(x + lxBottom);
        bottom->Y(y + lyBottom);
        bottom->Z(z + lzBottom);
        bottom->Vx(vx);
        bottom->Vy(vy);
        bottom->Vz(vz);
    }
}

void Cone::writeToFile(FILE *ft) const noexcept {
	Solid::WriteToFile(ft);
	fprintf(ft,"%e\t%e\t%e\t%e\n",h,r0,r1,dr);
	fprintf(ft,"%d\t%d\t%d\n",in,numTop,numBottom);
    Solid::WriteAccelerationFromFile(ft);
}

void Cone::writeOutFile(FILE *ft, int mode) const noexcept {
	Solid::WriteOutFile(ft,mode);
	if(mode == 0){
		fprintf(ft,"%e\t%e\t%e\t%e\n",h,r0,r1,dr);
		fprintf(ft,"%d\t%d\t%d\n",in,periodic,Ngb);
	}
	else{
		fwrite(&r0,sizeof(double),1,ft);
		fwrite(&r0,sizeof(double),1,ft);
		fwrite(&r1,sizeof(double),1,ft);
		fwrite(&dr,sizeof(double),1,ft);
		fwrite(&in,sizeof(double),1,ft);
		fwrite(&periodic,sizeof(double),1,ft);
		fwrite(&Ngb,sizeof(double),1,ft);
	}
}
