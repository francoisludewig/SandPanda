#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include "Contact.h"

class Sphere;
class Body;
class Plan;
class PlanR;
class Cone;
class Elbow;
class Data;

class ComputeForce {
public:
	static void InitForTimeStep(const int & Nsph, const int & Nbd , const int & Nct ,int & Npl, int & Nplr, int & Nco, int & Nelb, std::vector<Sphere> & sph, std::vector<Body> & bd, Contact *ct, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb) noexcept;

	static void InitForTimeStepOMP(const int & Nsph, const int & Nbd , const int & Nct , const int & Ncta , const int & Nctb , const int & Nctc ,int & Npl, int & Nplr, int & Nco, int & Nelb, std::vector<Sphere> & sph, std::vector<Body> & bd, Contact *ct, Contact *cta, Contact *ctb, Contact *ctc, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb) noexcept;

	static void Compute(Contact *ct, const int Nct,Data & dat) noexcept;

private:
	/* Fonction qui calcul la force globale au contact */
	static inline void ComputeContactForce(Contact *ctv, const double tx, const double ty, const double tz, const double T, const double N, double & Fx, double & Fy, double & Fz) noexcept {
		if(T != 0){
			ctv->tx = tx;
			ctv->ty = ty;
			ctv->tz = tz;
		}
		else{
			ctv->tx = 0.;
			ctv->ty = 0.;
			ctv->tz = 0.;
		}
		
		Fx = N*ctv->nx;
		Fy = N*ctv->ny;
		Fz = N*ctv->nz;
		
		if(T != 0){
			Fx += T*tx;
			Fy += T*ty;
			Fz += T*tz;
		}
		// Ajout de la force et du moment
		ctv->Fx = Fx;
		ctv->Fy = Fy;
		ctv->Fz = Fz;
	}
	
	/* Fonction qui calcul la vitesse de contact */
	static inline void computeVelocity(const double Vax,const double Vay,const double Vaz,const double Vbx,const double Vby,const double Vbz, Contact *ctv,
															double & Vn, double & Vt, double & Vtx, double & Vty, double & Vtz) noexcept {
		double vx,vy,vz;
		vx = Vax-Vbx;
		vy = Vay-Vby;
		vz = Vaz-Vbz;
		// Vn = produit scalaire de la vitesse relative et du vecteur normal au contact
		Vn = vx*ctv->nx+ vy*ctv->ny+ vz*ctv->nz;
		Vtx = vx - Vn*ctv->nx;
		Vty = vy - Vn*ctv->ny;
		Vtz = vz - Vn*ctv->nz;
		Vt = sqrt(Vtx*Vtx+Vty*Vty+Vtz*Vtz);
	}
	
	/* Fonction qui calcul les composantes N et T dans le cas d'un modele dynamique pour la force tangentielle */
	static inline void computeContactForceDynamic(double & N, double & T, Contact *ctv, const double k, const double g0, const double gt, const double mu, const double Vn, const double Vt, const double Vtx, const double Vty, const double Vtz, double & tx, double & ty, double & tz) noexcept {
		N = k*(-(ctv->delta)) - g0*Vn;
		if(N < 0) N = 0;
		T = -gt*Vt;
		double nt = fabs(T);
		if(nt > mu*N){
			ctv->Stick = 0;
			T = T/nt*mu*N;
		}
		else{
			ctv->Stick = 1;
		}
		if(Vt != 0){
			tx = Vtx/Vt;
			ty = Vty/Vt;
			tz = Vtz/Vt;
		}
		else{
			tx = 0.0;
			ty = 0.0;
			tz = 0.0;
		}
	}
	
	/* Fonction qui calcul les composantes N, T et Xsi dans le cas d'un modele statique pour la force tangentielle */
	static inline void computeContactForceStatic(Contact *ctv, Elongation & xsil, double & N, double & T, double & tx, double & ty, double & tz, const double Vn, const double Vt,
																				const double Vtx, const double Vty, const double Vtz, const double h, const double k, const double g0, const double muS, const double muD) noexcept {
		// Coefficient visqueux tangentiel nulle
		double gt = 0.2*g0;
		
		// Initialisation du status si pas d'elongation
		if(xsil.x == 0 && xsil.y == 0 && xsil.z == 0){
			//if(Vt == 0)
			xsil.status = 1;
			//else
			//	xsil.status = 0;
		}
		else{
			xsil.Rotate(ctv->nx, ctv->ny, ctv->nz);
		}
		// Choix du coefficient de friction en fonction du status
		double mu;
		if(xsil.status == 0)
			mu = muD;
		else
			mu = muS;
		
		// Calcul de la normale
		N = k*(-(ctv->delta)) - g0*Vn;
		if(N < 0) N = 0;
		
		// Calcul de la force tangentielle
		double tgx = -2./7.*k*xsil.x - gt*Vtx;
		double tgy = -2./7.*k*xsil.y - gt*Vty;
		double tgz = -2./7.*k*xsil.z - gt*Vtz;
		T = sqrt(tgx*tgx+tgy*tgy+tgz*tgz);
		
		if(T > mu*N){
			// Cas dynamique
			xsil.status = 0;
			ctv->Stick = 0;
			tx = tgx / T;
			ty = tgy / T;
			tz = tgz / T;
			T = muD*N;
			// Ajustement de xsi en accord avec la norme
			xsil.x = -7./(2.*k)*(T*tx + gt*Vtx);
			xsil.y = -7./(2.*k)*(T*ty + gt*Vty);
			xsil.z = -7./(2.*k)*(T*tz + gt*Vtz);
		}
		else{
			// Cas statique
			xsil.status = 1;
			ctv->Stick = 1;
			if(T > 0){
				tx = tgx / T;
				ty = tgy / T;
				tz = tgz / T;
			}
			else {
				T = 0;
			}
			// Incrementation de l'elongation
			xsil.x += Vtx*h;
			xsil.y += Vty*h;
			xsil.z += Vtz*h;
		}
	}
};
