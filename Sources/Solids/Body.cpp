#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "../../Includes/Solids/Body.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/BodySpecie.h"
#include "../../Includes/Configuration/Gravity.h"
#include "../../Includes/Solids/HollowBall.h"
#include "../../Includes/ComputingForce.h"
#include "../../Includes/Contact/ContactDetection.h"

Body::Body() :
sp(0), Ng(0), numl(0), Rmax(0), m(0), NhollowBall(0),ActiveRotation(0) {
	QuaternionToBase();
	Ine_1[0][0] = 0;
	Ine_1[0][1] = 0;
	Ine_1[0][2] = 0;
	Ine_1[1][0] = 0;
	Ine_1[1][1] = 0;
	Ine_1[1][2] = 0;
	Ine_1[2][0] = 0;
	Ine_1[2][1] = 0;
	Ine_1[2][2] = 0;
}

Body::~Body() {}

void Body::LoadFromFile(FILE *ft) noexcept {
	fscanf(ft,"%d\t%d\t",&sp,&NhollowBall);
	fscanf(ft,"%lf\t%lf\t%lf\t",&x,&y,&z);
	fscanf(ft,"%lf\t%lf\t%lf\t%lf\t",&q0,&q1,&q2,&q3);
	fscanf(ft,"%lf\t%lf\t%lf\t",&vx,&vy,&vz);
	fscanf(ft,"%lf\t%lf\t%lf\n",&wx,&wy,&wz);
	
	//q Ecriture de la base locale via le quaternion
	QuaternionToBase();
}

void Body::ReadStartStopFile(FILE *ft) noexcept {
	fscanf(ft,"%d\t%d\t",&sp,&NhollowBall);
	fscanf(ft,"%lf\t%lf\t%lf\t",&x,&y,&z);
	fscanf(ft,"%lf\t%lf\t%lf\t%lf\t",&q0,&q1,&q2,&q3);
	fscanf(ft,"%lf\t%lf\t%lf\t",&vx,&vy,&vz);
	fscanf(ft,"%lf\t%lf\t%lf\n",&wx,&wy,&wz);
	elongationManager.ReadFromFileForBody(ft);
		//q Ecriture de la base locale via le quaternion
		QuaternionToBase();
}

void Body::WriteToFile(FILE *ft,vector<Sphere> & sph) const noexcept {
	fprintf(ft,"%d\t%d\t",sp,NhollowBall);
	fprintf(ft,"%.15f\t%.15f\t%.15f\t",x,y,z);
	fprintf(ft,"%e\t%e\t%e\t%e\t",q0,q1,q2,q3);
	fprintf(ft,"%e\t%e\t%e\t",vx,vy,vz);
	fprintf(ft,"%e\t%e\t%e\n",wx,wy,wz);
	elongationManager.WriteToFileForBody(ft);
}

void Body::WriteOutFile(FILE *ft, int mode) const noexcept {
	if(mode == 0){
		fprintf(ft,"%d\t",sp);
		fprintf(ft,"%e\t%e\t%e\t",x,y,z);
		fprintf(ft,"%e\t%e\t%e\t%e\t",q0,q1,q2,q3);
		fprintf(ft,"%e\t%e\t%e\t",vx,vy,vz);
		fprintf(ft,"%e\t%e\t%e\n",wx,wy,wz);
	}
	else{
		fwrite(&sp, sizeof(int), 1, ft);
		fwrite(&x, sizeof(double), 1, ft);
		fwrite(&y, sizeof(double), 1, ft);
		fwrite(&z, sizeof(double), 1, ft);
		
		fwrite(&q0, sizeof(double), 1, ft);
		fwrite(&q1, sizeof(double), 1, ft);
		fwrite(&q2, sizeof(double), 1, ft);
		fwrite(&q3, sizeof(double), 1, ft);
		
		fwrite(&vx, sizeof(double), 1, ft);
		fwrite(&vy, sizeof(double), 1, ft);
		fwrite(&vz, sizeof(double), 1, ft);
		
		fwrite(&wx, sizeof(double), 1, ft);
		fwrite(&wy, sizeof(double), 1, ft);
		fwrite(&wz, sizeof(double), 1, ft);
		
	}
}

void Body::TimeStepInitialization() noexcept {
	Fx = 0;
	Fy = 0;
	Fz = 0;
	Mx = 0;
	My = 0;
	Mz = 0;
}

void Body::UpDateVelocity(double dt, Gravity & g) noexcept {
	double Mn,Mt,Ms,wn,wt,ws;
	Fx += m*g.ngx*g.G;
	Fy += m*g.ngy*g.G;
	Fz += m*g.ngz*g.G;

	vx += Fx/m*dt;
	vy += Fy/m*dt;
	vz += Fz/m*dt;
	if(ActiveRotation == 0){
		// Projection local
		Mn = Mx*nx+My*ny+Mz*nz;
		Mt = Mx*tx+My*ty+Mz*tz;
		Ms = Mx*sx+My*sy+Mz*sz;
		
		// Vitesse angulaire locale
		wn = (Ine_1[0][0]*Mn + Ine_1[0][1]*Mt + Ine_1[0][2]*Ms)*dt;
		wt = (Ine_1[1][0]*Mn + Ine_1[1][1]*Mt + Ine_1[1][2]*Ms)*dt;
		ws = (Ine_1[2][0]*Mn + Ine_1[2][1]*Mt + Ine_1[2][2]*Ms)*dt;
		
		// Incrementation dans la base globale
		wx += (wn*nx+wt*tx+ws*sx);
		wy += (wn*ny+wt*ty+ws*sy);
		wz += (wn*nz+wt*tz+ws*sz);
	}
}

void Body::Move(double dt) noexcept {
	double p0,p1,p2,p3;
	double ql0,ql1,ql2,ql3;
	double a,sa,ca;
	x += vx*dt;
	y += vy*dt;
	z += vz*dt;
	if(ActiveRotation == 0){
		a = sqrt(wx*wx+wy*wy+wz*wz)*dt;
		if(a != 0){
			sa = sin(a/2);
			ca = cos(a/2);
			p0 = ca;
			p1 = dt*wx/a*sa;
			p2 = dt*wy/a*sa;
			p3 = dt*wz/a*sa;
			
			ql0 = q0;
			ql1 = q1;
			ql2 = q2;
			ql3 = q3;
			
			if(p0 != 0 && ql0 != 0){
				q0 = ql0*p0 - ql1*p1 - ql2*p2 - ql3*p3;
				q1 = ql0*p1 + ql1*p0 - ql2*p3 + ql3*p2;
				q2 = ql0*p2 + ql1*p3 + ql2*p0 - ql3*p1;
				q3 = ql0*p3 - ql1*p2 + ql2*p1 + ql3*p0;
			}
			if(p0 != 0 && ql0 == 0){
				q0 = p0;
				q1 = p1;
				q2 = p2;
				q3 = p3;
			}
		}
		// Ecriture de la nouvelle base locale via le quaternion
		QuaternionToBase();
	}
}

void Body::UpDateLinkedSphere(vector<Sphere> & sph) noexcept {
	UpDateLinkedSphereTp();
	sph[numl].X(x);
	sph[numl].Y(y);
	sph[numl].Z(z);
}

void Body::UpDateLinkedSphereTp() noexcept {
	for(int j = 0 ; j < Ng ; j++){
		xg[j] = x + nx*xl[j] + tx*yl[j] + sx*zl[j];
		yg[j] = y + ny*xl[j] + ty*yl[j] + sy*zl[j];
		zg[j] = z + nz*xl[j] + tz*yl[j] + sz*zl[j];
	}
}

void Body::CancelVelocity() noexcept {
	vx = 0;
	vy = 0;
	vz = 0;
	wx = 0;
	wy = 0;
	wz = 0;
}

void Body::RandomVelocity(double V, double W) noexcept {
	double beta,alpha,rdm,Norme;
	beta=2*M_PI*(double)(rand()%RAND_MAX)/RAND_MAX;
	rdm=(double)(rand()%RAND_MAX)/RAND_MAX;
	alpha=acos(1-2*rdm);
	vz=cos(alpha);
	vx=cos(beta)*sin(alpha);
	vy=sin(beta)*sin(alpha);
	beta=2*M_PI*(double)(rand()%RAND_MAX)/RAND_MAX;
	rdm=(double)(rand()%RAND_MAX)/RAND_MAX;
	//TODO cos(aplha) = 1-2*rdm & sin(aplha) = sqrt(1-(1-2*rdm)^2)
	alpha=acos(1-2*rdm);
	wz=cos(alpha);
	wx=cos(beta)*sin(alpha);
	wy=sin(beta)*sin(alpha);
	Norme = sqrt(vx*vx+vy*vy+vz*vz);
	vx = vx/Norme*V;
	vy = vy/Norme*V;
	vz = vz/Norme*V;
	Norme = sqrt(wx*wx+wy*wy+wz*wz);
	wx = wx/Norme*W;
	wy = wy/Norme*W;
	wz = wz/Norme*W;
}

void Body::UploadSpecies(vector<BodySpecie> bdsp,vector<Sphere> & sph, int numero) noexcept {
	Ng = bdsp[sp].SphereCount();
	
	for(int j = 0 ; j < Ng ; j++){
		xl.push_back(bdsp[sp].SphereX(j));
		yl.push_back(bdsp[sp].SphereY(j));
		zl.push_back(bdsp[sp].SphereZ(j));
		r.push_back(bdsp[sp].SphereRadius(j));
		if(Rmax < r[j])Rmax = r[j];
			xg.push_back(x + nx*bdsp[sp].SphereX(j) + tx*bdsp[sp].SphereY(j) + sx*bdsp[sp].SphereZ(j));
			yg.push_back(y + ny*bdsp[sp].SphereX(j) + ty*bdsp[sp].SphereY(j) + sy*bdsp[sp].SphereZ(j));
			zg.push_back(z + nz*bdsp[sp].SphereX(j) + tz*bdsp[sp].SphereY(j) + sz*bdsp[sp].SphereZ(j));
			}
	
	Sphere sphl(numero, NhollowBall, bdsp[sp].GetFeretMax()/2.);
	sphl.X(x);
	sphl.Y(y);
	sphl.Z(z);
	sphl.Num(sph.size());
	numl = sph.size();
	sph.emplace_back(std::move(sphl));
	
	//Correction de la densite 7700 -> 1000
	//printf("m = %e\n",bdsp[sp].m);
	m = bdsp[sp].Mass();
	for(int i = 0 ; i < 3 ; i++){
		for(int j = 0 ; j < 3 ; j++){
			Ine_1[i][j] = bdsp[sp].Intertie(i,j);
		}
	}
}

void Body::SetActiveRotation(int na) noexcept {
	if(na == 0 || na == 1){
		ActiveRotation = na;
	}
}
