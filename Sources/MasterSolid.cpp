#include "../Includes/MasterSolid.h"

MasterSolid::MasterSolid() noexcept {
	x = 0;
	y = 0;
	z = 0;
	vx = 0;
	vy = 0;
	vz = 0;
	Fx = 0;
	Fy = 0;
	Fz = 0;
	Fcx = 0;
	Fcy = 0;
	Fcz = 0;
	pl = NULL;
	plr = NULL;
	co = NULL;
	Mass = 0;
}

void MasterSolid::initPlan(int a) noexcept {
	Npl = a;
	if(a != 0){
		pl = new Plan*[a];
		dxpl = new double[a];
		dypl = new double[a];
		dzpl = new double[a];
	}
	printf("MasterSolid : %d Plan\n",a);
}

void MasterSolid::initPlanR(int a) noexcept {
	Nplr = a;
	if(a != 0){
		plr = new PlanR*[a];
		dxplr = new double[a];
		dyplr = new double[a];
		dzplr = new double[a];
	}
	printf("MasterSolid : %d PlanR\n",a);
}

void MasterSolid::initCone(int a) noexcept {
	Nco = a;
	if(a != 0){
		co = new Cone *[a];
		dxco = new double[a];
		dyco = new double[a];
		dzco = new double[a];
	}
	printf("MasterSolid : %d Cone\n",a);
}

void MasterSolid::addPlan(std::vector<Plan> & pl2, int n, int m) noexcept {
	pl[m] = &pl2[n];
	Mass += pl2[n].Mass;
	x += pl2[n].x*pl2[n].Mass;
	y += pl2[n].y*pl2[n].Mass;
	z += pl2[n].z*pl2[n].Mass;
	Fcx += pl2[n].Fcx;
	Fcy += pl2[n].Fcy;
	Fcz += pl2[n].Fcz;
	printf("MasterSolid : add Plan %d\n",m);
}

void MasterSolid::addPlanR(std::vector<PlanR> & plr2, int n, int m) noexcept {
	plr[m] = &plr2[n];
	Mass += plr2[n].Mass;
	x += plr2[n].x*plr2[n].Mass;
	y += plr2[n].y*plr2[n].Mass;
	z += plr2[n].z*plr2[n].Mass;
	Fcx += plr2[n].Fcx;
	Fcy += plr2[n].Fcy;
	Fcz += plr2[n].Fcz;
	printf("MasterSolid : add PlanR %d\n",m);
}

void MasterSolid::addCone(std::vector<Cone> & co2, int n, int m) noexcept {
	co[m] = &co2[n];
	Mass += co2[n].Mass;
	x += co2[n].x*co2[n].Mass;
	y += co2[n].y*co2[n].Mass;
	z += co2[n].z*co2[n].Mass;
	Fcx += co2[n].Fcx;
	Fcy += co2[n].Fcy;
	Fcz += co2[n].Fcz;
	printf("MasterSolid : add Cone %d\n",m);
}

void MasterSolid::ComputePara() noexcept {
	x /= Mass;
	y /= Mass;
	z /= Mass;
	for(int i = 0 ; i < Npl ; i++){
		dxpl[i] = pl[i]->x-x;
		dypl[i] = pl[i]->y-y;
		dzpl[i] = pl[i]->z-z;
	}
	for(int i = 0 ; i < Nplr ; i++){
		dxplr[i] = plr[i]->x-x;
		dyplr[i] = plr[i]->y-y;
		dzplr[i] = plr[i]->z-z;
	}
	for(int i = 0 ; i < Nco ; i++){
		dxco[i] = co[i]->x-x;
		dyco[i] = co[i]->y-y;
		dzco[i] = co[i]->z-z;
	}
	printf("MasterSolid : cm = (%e,%e,%e)\n",x,y,z);
	printf("MasterSolid : Fc = (%e,%e,%e)\n",Fcx,Fcy,Fcz);
	printf("MasterSolid : mass = %e\n",Mass);
}

void MasterSolid::getForces() noexcept {
	Fx = Fcx;
	Fy = Fcy;
	Fz = Fcz;
	for(int i = 0 ; i < Npl ; i++){
		Fx += pl[i]->Fx;
		Fy += pl[i]->Fy;
		Fz += pl[i]->Fz;
	}
	for(int i = 0 ; i < Nplr ; i++){
		Fx += plr[i]->Fx;
		Fy += plr[i]->Fy;
		Fz += plr[i]->Fz;
	}
	for(int i = 0 ; i < Nco ; i++){
		Fx += co[i]->Fx;
		Fy += co[i]->Fy;
		Fz += co[i]->Fz;
	}
}

void MasterSolid::UpDateVelocity(double h) noexcept {
	if(Fcx != 0)
	vx += Fx/Mass*h;
	if(Fcy != 0)
	vy += Fy/Mass*h;
	if(Fcz != 0)
	vz += Fz/Mass*h;
}

void MasterSolid::Move(double h) noexcept {
	x += vx*h;
	y += vy*h;
	z += vz*h;
	UpDateSolid();
}

void MasterSolid::UpDateSolid() noexcept {
	for(int i = 0 ; i < Npl ; i++){
		pl[i]->x = x + dxpl[i];
		pl[i]->y = y + dypl[i];
		pl[i]->z = z + dzpl[i];
		pl[i]->vx = vx;
		pl[i]->vy = vy;
		pl[i]->vz = vz;
		pl[i]->wx = 0.;
		pl[i]->wy = 0.;
		pl[i]->wz = 0.;
		
	}
	for(int i = 0 ; i < Nplr ; i++){
		plr[i]->x = x + dxplr[i];
		plr[i]->y = y + dyplr[i];
		plr[i]->z = z + dzplr[i];
		plr[i]->vx = vx;
		plr[i]->vy = vy;
		plr[i]->vz = vz;
		plr[i]->wx = 0.;
		plr[i]->wy = 0.;
		plr[i]->wz = 0.;
	}
	for(int i = 0 ; i < Nco ; i++){
		co[i]->x = x + dxco[i];
		co[i]->y = y + dyco[i];
		co[i]->z = z + dzco[i];
		co[i]->vx = vx;
		co[i]->vy = vy;
		co[i]->vz = vz;
		co[i]->wx = 0.;
		co[i]->wy = 0.;
		co[i]->wz = 0.;
		co[i]->LimitUpdate();
	}
}
