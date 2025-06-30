#include <iostream>
#include <math.h>
#include <stdio.h>
#include <vector>

#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Configuration/Gravity.h"
#include "../../Includes/Solids/Body.h"

Sphere::Sphere() noexcept {
	// Initialisation des donnees des elongations
	r = 0.0005;
	m = 4./3.*2500*pow(r,3)*M_PI;
	rho = 2500;
	I = 2./5.*m*r*r;
	sp = 0;
	bodies = -9;
	NhollowBall = -9;
	autoIntegrate = true;
    isHollowBall = false;
	b = nullptr;
	tdl = nullptr;
	num = -9;
	ct_pl = 0;
}

Sphere::Sphere(const int bodies, const int nHollowBall, const double radius) noexcept : Sphere() {
	this->bodies = bodies;
	this->NhollowBall = nHollowBall;
	this->r = radius;
}


Sphere::Sphere(const double radius, const double mass, const double inertia) noexcept : Sphere() {
	this->r = radius;
	this->m = mass;
	this->I = inertia;
}

Sphere::~Sphere() noexcept {
	b = nullptr;
	tdl = nullptr;
}

int Sphere::count() const noexcept {
    if(bodies == -9 && !isHollowBall){
        return 0;
    }
    else
        return 1;
}

void Sphere::readFromFile(FILE *ft) noexcept {
	fscanf(ft,"%lf\t%lf\t%lf\n",&x,&y,&z);
	fscanf(ft,"%lf\t%lf\t%lf\t%lf\n",&q0,&q1,&q2,&q3);
	fscanf(ft,"%lf\t%lf\t%lf\n",&vx,&vy,&vz);
	fscanf(ft,"%lf\t%lf\t%lf\n",&wx,&wy,&wz);
	fscanf(ft,"%lf\t%lf\t%lf\t%d\t%d\n",&r,&m,&I,&sp,&NhollowBall);
	if(bodies == -9 && NhollowBall == -9){
		rho = m/(4./3.*M_PI*r*r*r);
	}
    r0 = r;
	if (q0 == 0 && q1 ==0 && q2 == 0 && q3 == 0)
		q0 = 1;
}

void Sphere::writeToFile(FILE *ft) const noexcept {
	if(bodies == -9 && !isHollowBall){
		fprintf(ft,"%.16e\t%.16e\t%.16e\n",x,y,z);
		fprintf(ft,"%.16e\t%.16e\t%.16e\t%.16e\n",q0,q1,q2,q3);
		fprintf(ft,"%.16e\t%.16e\t%.16e\n",vx,vy,vz);
		fprintf(ft,"%.16e\t%.16e\t%.16e\n",wx,wy,wz);
		fprintf(ft,"%e\t%e\t%e\t%d\t%d\n",r,m,I,sp,NhollowBall);
		elongationManager.writeToFile(ft);
	}
}

void Sphere::readStartStop(FILE *ft) noexcept {
	fscanf(ft,"%lf\t%lf\t%lf\n",&x,&y,&z);
	fscanf(ft,"%lf\t%lf\t%lf\t%lf\n",&q0,&q1,&q2,&q3);
	fscanf(ft,"%lf\t%lf\t%lf\n",&vx,&vy,&vz);
	fscanf(ft,"%lf\t%lf\t%lf\n",&wx,&wy,&wz);
	fscanf(ft,"%lf\t%lf\t%lf\t%d\t%d\n",&r,&m,&I,&sp,&NhollowBall);
	if(bodies == -9){
		rho = m/(4./3.*M_PI*r*r*r);
	}
	
	elongationManager.readFromFile(ft);
    r0 = r;
}

void Sphere::writeOutFile(FILE *ft, const int n, const int mode) const noexcept {
	if(bodies == -9 && !isHollowBall){
		// La masse et l'inertie sont dans le fichiers Export/grain.txt
		if(mode == 0){
			fprintf(ft,"%d\t%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%d\t%d\n",n,sp,r,x,y,z,q0,q1,q2,q3,vx,vy,vz,wx,wy,wz,bodies,NhollowBall);
		}
		else{
			fwrite(&n, sizeof(int), 1, ft);
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
			fwrite(&bodies, sizeof(int), 1, ft);
			fwrite(&NhollowBall, sizeof(int), 1, ft);
		}
	}
}

void Sphere::CancelVelocity() noexcept {
	vx = 0;
	vy = 0;
	vz = 0;
	wx = 0;
	wy = 0;
	wz = 0;
}

void Sphere::RandomVelocity(const double V, const double W) noexcept {
	double beta = 2 * M_PI * static_cast<double>(rand() % RAND_MAX) / RAND_MAX;
	double rdm = static_cast<double>(rand() % RAND_MAX) / RAND_MAX;
	double alpha = acos(1 - 2 * rdm);
	vz=cos(alpha);
	vx=cos(beta)*sin(alpha);
	vy=sin(beta)*sin(alpha);
	beta=2*M_PI*static_cast<double>(rand() % RAND_MAX)/RAND_MAX;
	rdm=static_cast<double>(rand() % RAND_MAX)/RAND_MAX;
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


void Sphere::initTimeStep() noexcept {
	Fx = 0.;
	Fy = 0.;
	Fz = 0.;
	Mx = 0.;
	My = 0.;
	Mz = 0.;
	elongationManager.InitXsi();
}

void Sphere::Melt(const double dt, const double vr) noexcept {
	if(autoIntegrate){
		r += r0*vr*dt;
		//m = 4./3.*M_PI*r*r*r*rho;
		I = 2./5.*m*r*r;
	}
}

void Sphere::Freeze(const double dt, const double vr) noexcept {
    // printf("r = %.15e -> %.15e  (dr = %e)\n",r,r+r0*vr*dt,r0*vr);
    r += r0*vr*dt;
    I = 2./5.*m*r*r;
}

void Sphere::upDateVelocity(const double dt, const Gravity &g, const double g0) noexcept {
	double gammab = g0;
	double gammabr = g0;
	if(autoIntegrate){
		Fx += m*g.ngx*g.G - vx*gammab;
		Fy += m*g.ngy*g.G - vy*gammab;
		Fz += m*g.ngz*g.G - vz*gammab;
		vx += Fx/m*dt;
		vy += Fy/m*dt;
		vz += Fz/m*dt;
		wx += (Mx - gammabr*r*r*wx)/I*dt;
		wy += (My - gammabr*r*r*wy)/I*dt;
		wz += (Mz - gammabr*r*r*wz)/I*dt;
	}
}

void Sphere::move(const double dt) noexcept {
	if(autoIntegrate){
		x += vx*dt;
		y += vy*dt;
		z += vz*dt;
		if(const double a = sqrt(wx*wx+wy*wy+wz*wz); a != 0){
			const double sa = sin(dt * a / 2);
			const double p0 = cos(dt * a / 2);
			const double p1 = wx / a * sa;
			const double p2 = wy / a * sa;
			const double p3 = wz / a * sa;

			const double ql0 = q0;
			const double ql1 = q1;
			const double ql2 = q2;
			const double ql3 = q3;
			q0 = ql0*p0 - ql1*p1 - ql2*p2 - ql3*p3;
			q1 = ql0*p1 + ql1*p0 - ql2*p3 + ql3*p2;
			q2 = ql0*p2 + ql1*p3 + ql2*p0 - ql3*p1;
			q3 = ql0*p3 - ql1*p2 + ql2*p1 + ql3*p0;
		}
	}
}

void Sphere::affiche() const noexcept {
	printf("Sphere : %d (r = %e)\n",num,r);
	printf("\tp = (%e,%e,%e)\n",x,y,z);
	printf("\tq = (%e,%e,%e,%e)\n",q0,q1,q2,q3);
	printf("\tv = (%e,%e,%e) & |v| = %e\n",vx,vy,vz,sqrt(vx*vx+vy*vy+vz*vz));
	printf("\tw = (%e,%e,%e) & |w| = %e\n\n",wx,wy,wz,sqrt(wx*wx+wy*wy+wz*wz));
}

void Sphere::sphereLinking(std::vector<Sphere> & sph,  std::vector<Body> & bd) noexcept {
	for(int i = 0 ; i < sph.size() ; ++i){
		sph[i].Num(i);
		if(sph[i].Bodies() != -9){
			sph[i].SetBody(&bd[sph[i].Bodies()]);
			sph[i].autoIntegrate = false;
		}
	}
}

void Sphere::ComputeCTD(const double R, const double w, const double t) noexcept {
    Fx += m*R*w*w*cos(w*t);
    Fy += m*R*w*w*sin(w*t);
}

void Sphere::ComputeRD(const double R, const double w, const double t) noexcept {
    Fx += m * ( R*w*w*cos(w*t) + x*w*w + 2*(-w)*vy );
    Fy += m * ( R*w*w*sin(w*t) + y*w*w - 2*(-w)*vx );
}

int Sphere::NoBodies() const noexcept {
	if(bodies == -9)
		return 1;
	else
		return 0;
}

int Sphere::NoAvatar() const noexcept {
    if(isHollowBall)
        return 0;
    else
        return 1;
}

