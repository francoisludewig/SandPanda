#include <iostream>
#include <cmath>
#include <cstdio>
#include "../../Includes/Solids/Body.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/BodySpecie.h"
#include "../../Includes/Configuration/Gravity.h"
#include "../../Includes/ComputingForce.h"

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

Body::~Body() = default;

void Body::LoadFromFile(FILE *ft) noexcept {
	fscanf(ft,"%d\t%d\t",&sp,&NhollowBall);
	fscanf(ft,"%lf\t%lf\t%lf\t",&x,&y,&z);
	fscanf(ft,"%lf\t%lf\t%lf\t%lf\t",&q0,&q1,&q2,&q3);
	fscanf(ft,"%lf\t%lf\t%lf\t",&vx,&vy,&vz);
	fscanf(ft,"%lf\t%lf\t%lf\n",&wx,&wy,&wz);

	if (q0 == 0 && q1 ==0 && q2 == 0 && q3 == 0)
		q0 = 1;
	//q Ecriture de la base locale via le quaternion
	QuaternionToBase();
}

void Body::ReadStartStopFile(FILE *ft) noexcept {
	fscanf(ft,"%d\t%d\t",&sp,&NhollowBall);
	fscanf(ft,"%lf\t%lf\t%lf\t",&x,&y,&z);
	fscanf(ft,"%lf\t%lf\t%lf\t%lf\t",&q0,&q1,&q2,&q3);
	fscanf(ft,"%lf\t%lf\t%lf\t",&vx,&vy,&vz);
	fscanf(ft,"%lf\t%lf\t%lf\n",&wx,&wy,&wz);
	elongationManager.readFromFile(ft);
		//q Ecriture de la base locale via le quaternion
		QuaternionToBase();
}

void Body::WriteToFile(FILE *ft,std::vector<Sphere> & sph) const noexcept {
	fprintf(ft,"%d\t%d\t",sp,NhollowBall);
	fprintf(ft,"%.15f\t%.15f\t%.15f\t",x,y,z);
	fprintf(ft,"%e\t%e\t%e\t%e\t",q0,q1,q2,q3);
	fprintf(ft,"%e\t%e\t%e\t",vx,vy,vz);
	fprintf(ft,"%e\t%e\t%e\n",wx,wy,wz);
	elongationManager.writeToFile(ft);
}

void Body::WriteOutFile(FILE *ft, const int mode) const noexcept {
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
	elongationManager.InitXsi();
}

void Body::UpDateVelocity(const double dt, const Gravity & g) noexcept {
	Fx += m*g.ngx*g.G;
	Fy += m*g.ngy*g.G;
	Fz += m*g.ngz*g.G;

	vx += Fx/m*dt;
	vy += Fy/m*dt;
	vz += Fz/m*dt;
	if(ActiveRotation == 0){
		// Projection local
		const double Mn = Mx*nx+My*ny+Mz*nz;
		const double Mt = Mx*tx+My*ty+Mz*tz;
		const double Ms = Mx*sx+My*sy+Mz*sz;
		
		// Vitesse angulaire locale
		const double wn = (Ine_1[0][0]*Mn + Ine_1[0][1]*Mt + Ine_1[0][2]*Ms)*dt;
		const double wt = (Ine_1[1][0]*Mn + Ine_1[1][1]*Mt + Ine_1[1][2]*Ms)*dt;
		const double ws = (Ine_1[2][0]*Mn + Ine_1[2][1]*Mt + Ine_1[2][2]*Ms)*dt;
		
		// Incrementation dans la base globale
		wx += (wn*nx+wt*tx+ws*sx);
		wy += (wn*ny+wt*ty+ws*sy);
		wz += (wn*nz+wt*tz+ws*sz);
	}
}

void Body::Move(const double dt) noexcept {
	x += vx*dt;
	y += vy*dt;
	z += vz*dt;
	if(ActiveRotation == 0){
		if(const double a = sqrt(wx*wx+wy*wy+wz*wz)*dt; a != 0){
			const double sa = sin(a/2);
			const double ca = cos(a/2);
			const double p0 = ca;
			const double p1 = dt*wx/a*sa;
			const double p2 = dt*wy/a*sa;
			const double p3 = dt*wz/a*sa;
			
			const double ql0 = q0;
			const double ql1 = q1;
			const double ql2 = q2;
			const double ql3 = q3;
			
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

void Body::UpDateLinkedSphere(std::vector<Sphere> & sph) noexcept {
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

void Body::RandomVelocity(const double V, const double W) noexcept {
	double beta = 2 * M_PI * static_cast<double>(rand() % RAND_MAX) / RAND_MAX;
	double rdm = static_cast<double>(rand() % RAND_MAX) / RAND_MAX;
	double alpha = acos(1 - 2 * rdm);
	vz=cos(alpha);
	vx=cos(beta)*sin(alpha);
	vy=sin(beta)*sin(alpha);
	beta=2*M_PI*static_cast<double>(rand() % RAND_MAX)/RAND_MAX;
	rdm=static_cast<double>(rand() % RAND_MAX)/RAND_MAX;
	//TODO cos(aplha) = 1-2*rdm & sin(aplha) = sqrt(1-(1-2*rdm)^2)
	alpha=acos(1-2*rdm);
	wz=cos(alpha);
	wx=cos(beta)*sin(alpha);
	wy=sin(beta)*sin(alpha);
	double Norm = sqrt(vx * vx + vy * vy + vz * vz);
	vx = vx/Norm*V;
	vy = vy/Norm*V;
	vz = vz/Norm*V;
	Norm = sqrt(wx*wx+wy*wy+wz*wz);
	wx = wx/Norm*W;
	wy = wy/Norm*W;
	wz = wz/Norm*W;
}

void Body::UploadSpecies(std::vector<BodySpecie> bdsp, std::vector<Sphere> & sph, int numero) noexcept {
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
	sphl.Num(static_cast<int>(sph.size()));
	numl = static_cast<int>(sph.size());
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

void Body::SetActiveRotation(const int na) noexcept {
	if(na == 0 || na == 1){
		ActiveRotation = na;
	}
}
