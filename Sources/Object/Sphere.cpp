#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "../../Includes/Object/Sphere.h"
#include "../../Includes/Configuration/Gravity.h"
#include "../../Includes/Contact/Elongation.h"
#include "../../Includes/Object/Body.h"

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
	b = NULL;
	tdl = NULL;
	num = -9;
	ct_pl = 0;
	// Elongation
    xsi.reserve(maxContact);
    tp_xsi.reserve(maxContact);
    type.reserve(maxContact);
    tp_type.reserve(maxContact);
    neighbourNumber.reserve(maxContact);
    tp_neighbourNumber.reserve(maxContact);
    bodyNumber.reserve(maxContact);
    tp_bodyNumber.reserve(maxContact);

	for(int i = 0 ; i < maxContact ; i++){
        neighbourNumber.push_back(-9);
        type.push_back(-1);
        Elongation elongation;
        xsi.push_back(elongation);
        tp_neighbourNumber.push_back(-9);
        tp_type.push_back(-1);
        Elongation tp_elongation;
        tp_xsi.push_back(tp_elongation);
	}

	Nneighbour = 0;
	Nneighbour2 = 0;
}

Sphere::~Sphere() noexcept {
	b = NULL;
	tdl = NULL;
}

double Sphere::Radius() const noexcept {
	return r;
}
double Sphere::Rho() const noexcept {
	return rho;
}

double Sphere::Mass() const noexcept {
	return m;
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
}

void Sphere::writeToFile(FILE *ft) const noexcept {
	if(bodies == -9 && !isHollowBall){
		fprintf(ft,"%.15f\t%.15f\t%.15f\n",x,y,z);
		fprintf(ft,"%e\t%e\t%e\t%e\n",q0,q1,q2,q3);
		fprintf(ft,"%e\t%e\t%e\n",vx,vy,vz);
		fprintf(ft,"%e\t%e\t%e\n",wx,wy,wz);
		fprintf(ft,"%e\t%e\t%e\t%d\t%d\n",r,m,I,sp,NhollowBall);
		fprintf(ft,"%d\n",Nneighbour);
		for(int i = 0 ; i < Nneighbour ; i++)
			fprintf(ft,"%d\t%d\t%e\t%e\t%e\n",neighbourNumber[i],type[i],xsi[i].x,xsi[i].y,xsi[i].z);
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
	
	fscanf(ft,"%d\n",&Nneighbour2);
	//Nneighbour2+=maxContact;
	for(int i = 0 ; i < Nneighbour2 ; i++)
		fscanf(ft,"%d\t%d\t%lf\t%lf\t%lf\n",&neighbourNumber[i],&type[i],&xsi[i].x,&xsi[i].y,&xsi[i].z);
    r0 = r;
}

void Sphere::writeOutFile(FILE *ft, int n, int mode) const noexcept {
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

void Sphere::RandomVelocity(double V, double W) noexcept {
	double beta,alpha,rdm,Norme;
	beta=2*M_PI*(double)(rand()%RAND_MAX)/RAND_MAX;
	rdm=(double)(rand()%RAND_MAX)/RAND_MAX;
	alpha=acos(1-2*rdm);
	vz=cos(alpha);
	vx=cos(beta)*sin(alpha);
	vy=sin(beta)*sin(alpha);
	beta=2*M_PI*(double)(rand()%RAND_MAX)/RAND_MAX;
	rdm=(double)(rand()%RAND_MAX)/RAND_MAX;
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


void Sphere::initTimeStep() noexcept {
	Fx = 0.;
	Fy = 0.;
	Fz = 0.;
	Mx = 0.;
	My = 0.;
	Mz = 0.;
}

void Sphere::Freeze(double dt, double vr) noexcept {
    // printf("r = %.15e -> %.15e  (dr = %e)\n",r,r+r0*vr*dt,r0*vr);
    r += r0*vr*dt;
    I = 2./5.*m*r*r;
}

void Sphere::upDateVelocity(double dt, Gravity & g, double g0) noexcept {
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

void Sphere::move(double dt) noexcept {
	double a,sa;
	double p0,p1,p2,p3;
	double ql0,ql1,ql2,ql3;
	if(autoIntegrate){
		x += vx*dt;
		y += vy*dt;
		z += vz*dt;
		a = sqrt(wx*wx+wy*wy+wz*wz);
		if(a != 0){
			sa = sin(dt*a/2);
			p0 = cos(dt*a/2);
			p1 = wx/a*sa;
			p2 = wy/a*sa;
			p3 = wz/a*sa;
			
			ql0 = q0;
			ql1 = q1;
			ql2 = q2;
			ql3 = q3;

            if(q0 == 0 && q1 == 0 && q2 ==0 && q3 == 0) {
                q0 = p0;
                q1 = p1;
                q2 = p2;
                q3 = p3;
            } else {
                q0 = ql0 * p0 - ql1 * p1 - ql2 * p2 - ql3 * p3;
                q1 = ql0 * p1 + ql1 * p0 - ql2 * p3 + ql3 * p2;
                q2 = ql0 * p2 + ql1 * p3 + ql2 * p0 - ql3 * p1;
                q3 = ql0 * p3 - ql1 * p2 + ql2 * p1 + ql3 * p0;
            }
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

void Sphere::setRadius(double alpha) noexcept {
	if(autoIntegrate){
		r = r*alpha;
		m = 4./3.*rho*pow(r,3)*M_PI;
		I = 2./5.*m*r*r;
		vx = 0;
		vy = 0;
		vz = 0;
		wx = 0;
		wy = 0;
		wz = 0;
	}
}

void Sphere::sphereLinking(int & Nsph , vector<Sphere> & sph,  vector<Body> & bd) noexcept {
	for(int i = 0 ; i < Nsph ; i++){
		sph[i].num = i;
		if(sph[i].bodies != -9){
			sph[i].b = &bd[sph[i].bodies];
			sph[i].autoIntegrate = 0;
		}
	}
}

int Sphere::Num() const noexcept {
	return num;
}

// TODO Implement swapable container with RAII
void Sphere::InitXsi() noexcept {
    std::swap(xsi, tp_xsi);
    std::swap(neighbourNumber, tp_neighbourNumber);
    std::swap(type, tp_type);
    std::swap(bodyNumber, tp_bodyNumber);
	Nneighbour = Nneighbour2;
	Nneighbour2 = 0;
}

void Sphere::AddXsi(Elongation& e, int n , int t, int nob) noexcept {
	tp_xsi[Nneighbour2] = e;
	tp_neighbourNumber[Nneighbour2] = n;
	tp_type[Nneighbour2] = t;
	tp_bodyNumber[Nneighbour2] = nob;
	Nneighbour2++;
}

Elongation Sphere::FoundIt(int n, int t, int nob) const noexcept {
	static Elongation e;
	for(int i = 0 ; i < Nneighbour ; i++){
		if(neighbourNumber[i] == n && type[i] == t){
			if(type[i] != 10){
				return xsi[i];
			}
			else{
				if(nob == bodyNumber[i])
					return xsi[i];
			}
        }
	}
	return(e);
}

void Sphere::ComputeCTD(double R, double w, double t) noexcept {
    Fx += m*R*w*w*cos(w*t);
    Fy += m*R*w*w*sin(w*t);
}

void Sphere::ComputeRD(double R, double w, double t) noexcept {
    Fx += m * ( R*w*w*cos(w*t) + x*w*w + 2*(-w)*vy );
    Fy += m * ( R*w*w*sin(w*t) + y*w*w - 2*(-w)*vx );
}

double Sphere::radius() const noexcept {
	return r;
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

double Sphere::getFx() const noexcept {
	return Fx;
}

double Sphere::getFy() const noexcept {
	return Fy;
}

double Sphere::getFz() const noexcept {
	return Fz;
}

double Sphere::getRho() const noexcept {
	return rho;
}

int Sphere::HollowballNum() const noexcept {
    return NhollowBall;
}

void Sphere::setIsHollowBall(bool a) noexcept {
    isHollowBall = a;
}
