#include "../Includes/Cone.h"
#include "../Includes/PlanR.h"
#include <vector>

Cone::Cone() noexcept :Solid(){
	r0 = 0.25;
	r1 = 0.25;
	dr = 0;
	h = 0.5;
	in = 0;
	periodic = -9;
    numTop = -9;
    numBottom = -9;
    top = NULL;
    bottom = NULL;
}

void Cone::readFromFile(FILE *ft) noexcept{
	Solid::LoadFromFile(ft);
	fscanf(ft,"%lf\t%lf\t%lf\t%lf\n",&h,&r0,&r1,&dr);
	fscanf(ft,"%d\t%d\t%d\n",&in,&numTop,&numBottom);
    Solid::LoadAccelerationFromFile(ft);
}

void Cone::LimitLink(std::vector<PlanR> & plr) noexcept {
    if(numTop != -9){
        top = &plr[numTop];
        top->Force = 1;
        lxTop = top->X() - x;
        lyTop = top->Y() - y;
        lzTop = top->Z() - z;
    }
    if(numBottom != -9){
        bottom = &plr[numBottom];
        bottom->Force = 1;
        lxBottom = bottom->X() - x;
        lyBottom = bottom->Y() - y;
        lzBottom = bottom->Z() - z;
    }
}


void Cone::LimitForce() noexcept {
    if(top != NULL){
        Fx += top->GetFx();
        Fy += top->GetFy();
        Fz += top->GetFz();
    }
    if(bottom != NULL){
        Fx += bottom->GetFx();
        Fy += bottom->GetFy();
        Fz += bottom->GetFz();
    }
}

void Cone::LimitUpdate() noexcept {
    if(top != NULL){
        top->X(x + lxTop);
        top->Y(y + lyTop);
        top->Z(z + lzTop);
        top->Vx(vx);
        top->Vy(vy);
        top->Vz(vz);
    }
    if(bottom != NULL){
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
