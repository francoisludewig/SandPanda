#include <iostream>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <random>
#include "../../Includes/Object/Sphere.h"
#include "../../Includes/Configuration/Gravity.h"
#include "../../Includes/Contact/Elongation.h"
#include "../../Includes/Contact/ContactIdentifier.h"
#include "../../Includes/Object/Body.h"

Sphere::Sphere() noexcept : MechanicalPoint() {
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
	// Elongation
    /*
    xsi.reserve(maxContact);
    tp_xsi.reserve(maxContact);
    type.reserve(maxContact);
    tp_type.reserve(maxContact);
    neighbourNumber.reserve(maxContact);
    tp_neighbourNumber.reserve(maxContact);
    bodyNumber.reserve(maxContact);
    tp_bodyNumber.reserve(maxContact);

    std::fill(type.begin(), type.end(), -1);
    std::fill(tp_type.begin(), tp_type.end(), -1);
    std::fill(neighbourNumber.begin(), neighbourNumber.end(), -9);
    std::fill(tp_neighbourNumber.begin(), tp_neighbourNumber.end(), -9);

	for(int i = 0 ; i < maxContact ; i++){
        Elongation elongation;
        xsi.push_back(elongation);
        Elongation tp_elongation;
        tp_xsi.push_back(tp_elongation);
	}

	Nneighbour = 0;
	Nneighbour2 = 0;
     */
}

Sphere::~Sphere() noexcept {
	b = nullptr;
	tdl = nullptr;
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
        elongationManager.writeToFile(ft);
        /*
		fprintf(ft,"%d\n",Nneighbour);
		for(int i = 0 ; i < Nneighbour ; i++)
			fprintf(ft,"%d\t%d\t%e\t%e\t%e\n",neighbourNumber[i],type[i],xsi[i].x,xsi[i].y,xsi[i].z);
        */
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
    /*
	fscanf(ft,"%d\n",&Nneighbour2);
	//tp_count+=maxContact;
	for(int i = 0 ; i < Nneighbour2 ; i++)
		fscanf(ft,"%d\t%d\t%lf\t%lf\t%lf\n",&neighbourNumber[i],&type[i],&xsi[i].x,&xsi[i].y,&xsi[i].z);
    */
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

void Sphere::RandomVelocity(double V, double W) noexcept {
    std::random_device rd;  // Graine aléatoire
    std::mt19937 gen(rd()); // Générateur Mersenne Twister
    std::uniform_real_distribution<double> distrib(0.0, 1.0);
	double beta,alpha,rdm,speed;
	beta=2*M_PI*distrib(gen);
	rdm=distrib(gen);
	alpha=acos(1-2*rdm);
	vz=cos(alpha);
	vx=cos(beta)*sin(alpha);
	vy=sin(beta)*sin(alpha);
	beta=2*M_PI*distrib(gen);
	rdm=distrib(gen);
	alpha=acos(1-2*rdm);
	wz=cos(alpha);
	wx=cos(beta)*sin(alpha);
	wy=sin(beta)*sin(alpha);
    speed = sqrt(vx * vx + vy * vy + vz * vz);
	vx = vx / speed * V;
	vy = vy / speed * V;
	vz = vz / speed * V;
    speed = sqrt(wx * wx + wy * wy + wz * wz);
	wx = wx / speed * W;
	wy = wy / speed * W;
	wz = wz / speed * W;
}

void Sphere::Freeze(double dt, double vr) noexcept {
    // printf("r = %.15e -> %.15e  (dr = %e)\n",r,r+r0*vr*dt,r0*vr);
    r += r0*vr*dt;
    I = 2./5.*m*r*r;
}

void Sphere::upDateVelocity(double dt, Gravity & g, double g0) noexcept {
	double gamma_b = g0;
	double gamma_br = g0;
	if(autoIntegrate){
		Fx += m*g.ngx*g.G - vx * gamma_b;
		Fy += m*g.ngy*g.G - vy * gamma_b;
		Fz += m*g.ngz*g.G - vz * gamma_b;
		vx += Fx/m*dt;
		vy += Fy/m*dt;
		vz += Fz/m*dt;
		wx += (Mx - gamma_br * r * r * wx) / I * dt;
		wy += (My - gamma_br * r * r * wy) / I * dt;
		wz += (Mz - gamma_br * r * r * wz) / I * dt;
	}
}

void Sphere::move(double dt) noexcept {
    if(autoIntegrate) {
        MechanicalPoint::move(dt);
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
			sph[i].autoIntegrate = false;
		}
	}
}

int Sphere::Num() const noexcept {
	return num;
}

// TODO Implement swapable container with RAII
void Sphere::InitXsi() noexcept {
    elongationManager.InitXsi();
    /*
    std::swap(xsi, tp_xsi);
    std::swap(neighbourNumber, tp_neighbourNumber);
    std::swap(type, tp_type);
    std::swap(bodyNumber, tp_bodyNumber);
	Nneighbour = Nneighbour2;
	Nneighbour2 = 0;
    */
}

void Sphere::AddXsi(Elongation& e, uint64_t contactIdentifier) noexcept {
    elongationManager.AddXsi(e, contactIdentifier);
}

void Sphere::AddXsi(Elongation& e, int n , int t, int nob) noexcept {
    /*if(t==0 && nob == -9) // SPH SPH
        elongationManager.AddXsi(e, ContactIdentifier::computeIdentifier(n));
    else if(t != 0 && nob == -9) // SPH SOLID
        elongationManager.AddXsi(e, ContactIdentifier::computeIdentifier(n, t));
    else if(t != 0 && nob != 0)
        elongationManager.AddXsi(e, ContactIdentifier::computeIdentifier(n, t, nob));
*/
    /*
    tp_xsi[Nneighbour2] = e;
	tp_neighbourNumber[Nneighbour2] = n;
	tp_type[Nneighbour2] = t;
	tp_bodyNumber[Nneighbour2] = nob;
	Nneighbour2++;
     */
}

Elongation Sphere::FoundIt(uint64_t contactIdentifier) const noexcept {
    return elongationManager.FoundIt(contactIdentifier);
}

Elongation Sphere::FoundIt(int n, int t, int nob) const noexcept {
	static Elongation e;
	/*

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
	 */
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
