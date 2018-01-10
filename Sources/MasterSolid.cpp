#include "../Includes/MasterSolid.h"

MasterSolid::MasterSolid() noexcept :
	x(0), y(0), z(0), vx(0), vy(0), vz(0),
	Fx(0), Fy(0), Fz(0), Fcx(0), Fcy(0), Fcz(0),
	Mass(0), Npl(0),Nplr(0),Nco(0) {}

void MasterSolid::addPlan(std::vector<Plan> & pl2, int n, int m) noexcept {
	pl.push_back(&pl2[n]);
	//pl[m] = &pl2[n];
	Mass += pl2[n].GetMass();
	x += pl2[n].X()*pl2[n].GetMass();
	y += pl2[n].Y()*pl2[n].GetMass();
	z += pl2[n].Z()*pl2[n].GetMass();
	Fcx += pl2[n].GetFcx();
	Fcy += pl2[n].GetFcy();
	Fcz += pl2[n].GetFcz();
	printf("MasterSolid : add Plan %d\n",m);
}

void MasterSolid::addPlanR(std::vector<PlanR> & plr2, int n, int m) noexcept {
	plr.push_back(&plr2[n]);
	//plr[m] = &plr2[n];
	Mass += plr2[n].GetMass();
	x += plr2[n].X()*plr2[n].GetMass();
	y += plr2[n].Y()*plr2[n].GetMass();
	z += plr2[n].Z()*plr2[n].GetMass();
	Fcx += plr2[n].GetFcx();
	Fcy += plr2[n].GetFcy();
	Fcz += plr2[n].GetFcz();
	printf("MasterSolid : add PlanR %d\n",m);
}

void MasterSolid::addCone(std::vector<Cone> & co2, int n, int m) noexcept {
	co.push_back(&co2[n]);
	//co[m] = &co2[n];
	Mass += co2[n].GetMass();
	x += co2[n].X()*co2[n].GetMass();
	y += co2[n].Y()*co2[n].GetMass();
	z += co2[n].Z()*co2[n].GetMass();
	Fcx += co2[n].GetFcx();
	Fcy += co2[n].GetFcy();
	Fcz += co2[n].GetFcz();
	printf("MasterSolid : add Cone %d\n",m);
}

void MasterSolid::ComputePara() noexcept {
	x /= Mass;
	y /= Mass;
	z /= Mass;
	for(int i = 0 ; i < Npl ; i++){
		dxpl[i] = pl[i]->X()-x;
		dypl[i] = pl[i]->Y()-y;
		dzpl[i] = pl[i]->Z()-z;
	}
	for(int i = 0 ; i < Nplr ; i++){
		dxplr[i] = plr[i]->X()-x;
		dyplr[i] = plr[i]->Y()-y;
		dzplr[i] = plr[i]->Z()-z;
	}
	for(int i = 0 ; i < Nco ; i++){
		dxco[i] = co[i]->X()-x;
		dyco[i] = co[i]->Y()-y;
		dzco[i] = co[i]->Z()-z;
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
		Fx += pl[i]->GetFx();
		Fy += pl[i]->GetFy();
		Fz += pl[i]->GetFz();
	}
	for(int i = 0 ; i < Nplr ; i++){
		Fx += plr[i]->GetFx();
		Fy += plr[i]->GetFy();
		Fz += plr[i]->GetFz();
	}
	for(int i = 0 ; i < Nco ; i++){
		Fx += co[i]->GetFx();
		Fy += co[i]->GetFy();
		Fz += co[i]->GetFz();
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
		pl[i]->X(x + dxpl[i]);
		pl[i]->Y(y + dypl[i]);
		pl[i]->Z(z + dzpl[i]);
		pl[i]->Vx(vx);
		pl[i]->Vy(vy);
		pl[i]->Vz(vz);
		pl[i]->Wx(0.);
		pl[i]->Wy(0.);
		pl[i]->Wz(0.);
		
	}
	for(int i = 0 ; i < Nplr ; i++){
		plr[i]->X(x + dxplr[i]);
		plr[i]->Y(y + dyplr[i]);
		plr[i]->Z(z + dzplr[i]);
		plr[i]->Vx(vx);
		plr[i]->Vy(vy);
		plr[i]->Vz(vz);
		plr[i]->Wx(0.);
		plr[i]->Wy(0.);
		plr[i]->Wz(0.);
	}
	for(int i = 0 ; i < Nco ; i++){
		co[i]->X(x + dxco[i]);
		co[i]->Y(y + dyco[i]);
		co[i]->Z(z + dzco[i]);
		co[i]->Vx(vx);
		co[i]->Vy(vy);
		co[i]->Vz(vz);
		co[i]->Wx(0.);
		co[i]->Wy(0.);
		co[i]->Wz(0.);
		co[i]->LimitUpdate();
	}
}
