#include "../../Includes/Contact/ComputingForce.h"

#include "../../Includes/Object/Velocity.h"
#include "../../Includes/Configuration/Gravity.h"
#include "../../Includes/Object/Plan.h"
#include "../../Includes/Object/PlanR.h"
#include "../../Includes/Object/Cone.h"
#include "../../Includes/Object/Elbow.h"
#include "../../Includes/Object/Sphere.h"
#include "../../Includes/Object/Body.h"
#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Repository/ReadWrite.h"
#include "../../Includes/Contact/ContactDetection.h"
#include "../../Includes/Contact/Elongation.h"
#include "../../Includes/Configuration/Data.h"

/* Fonction qui initialise tous les obejts avant chaque etape de calcul */
void ComputeForce::InitForTimeStep(const int & Nsph, const int & Nbd , const int & Nct ,int & Npl, int & Nplr, int & Nco, int & Nelb, vector<Sphere> & sph, vector<Body> & bd, Contact *ct, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb) noexcept {
	Sphere *sphl = &sph[0];
	for(int i = 0 ; i < Nsph ; i++){
        sphl->resetForceAndMomentum();
		sphl++;
	}
	for(int i = 0 ; i < Nbd ; i++)
		bd[i].TimeStepInitialization();
	for(int i = 0 ; i < Nct ; i++)
		ct[i].TimeStepInitialization();	
	for(int i = 0 ; i < Npl ; i++)
        pl[i].resetForceAndMomentum();
	for(int i = 0 ; i < Nplr ; i++)
        plr[i].resetForceAndMomentum();
	for(int i = 0 ; i < Nco ; i++)
        co[i].resetForceAndMomentum();
}

/* Fonction qui initialise tous les obejts avant chaque etape de calcul en version parallel OMP */
void ComputeForce::InitForTimeStepOMP(const int & Nsph, const int & Nbd , const int & Nct , const int & Ncta , const int & Nctb , const int & Nctc ,int & Npl, int & Nplr, int & Nco, int & Nelb, vector<Sphere> & sph, vector<Body> & bd, Contact *ct, Contact *cta, Contact *ctb, Contact *ctc, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb) noexcept {
	// Sphere
	for(int i = 0 ; i < Nsph ; i++)
        sph[i].resetForceAndMomentum();
	// Bodies
	for(int i = 0 ; i < Nbd ; i++)
		bd[i].TimeStepInitialization();
	// Ct
	for(int i = 0 ; i < Nct ; i++)
		ct[i].TimeStepInitialization();	
	// Cta si para 2
	for(int i = 0 ; i < Ncta ; i++)
		cta[i].TimeStepInitialization();	
	// Ctb si para 4
	for(int i = 0 ; i < Nctb ; i++)
		ctb[i].TimeStepInitialization();	
	// Ctc si para 4
	for(int i = 0 ; i < Nctc ; i++)
		ctc[i].TimeStepInitialization();	
	// Plan
	for(int i = 0 ; i < Npl ; i++)
        pl[i].resetForceAndMomentum();
	// Disque
	for(int i = 0 ; i < Nplr ; i++)
        plr[i].resetForceAndMomentum();
    // Cone
	for(int i = 0 ; i < Nco ; i++)
        co[i].resetForceAndMomentum();

}

/* Fonction qui calcul les forces pour l'ensemble des contacts du tableau ct */
void ComputeForce::Compute(Contact *ct, const int Nct, Data & dat) noexcept {
	double lax,lay,laz,lbx,lby,lbz;
	double Vax,Vay,Vaz,Vbx,Vby,Vbz;
	double wbx,wby,wbz;
	double Vn,Vtx,Vty,Vtz,Vt;
	double tx = 0,ty = 0,tz = 0;
	double meff,g0,N =0,T = 0,gt;
	double Fx,Fy,Fz;
	Elongation xsi;	
	//Data from dat
	double en = dat.en;
	double mu = dat.mu;
	double k = dat.k;
	double TIME = dat.TIME;
	double h = dat.dt;
	int ModelTg = dat.modelTg;
	Sphere *a,*b;
	Body *ba,*bb;
	int na,nb;
	Plan *p;
	PlanR *pr;
	Cone *cne;
	Elbow *elw;
	Contact *ctl = ct;
	
	for(int i = 0 ; i < Nct ; i++){
		N = 0;
		T = 0;
		switch(ctl->type){
			case 0:
				// Cas du contact entre deux spheres
				a = ctl->sa;
				b = ctl->sb;
			    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = -a->r*ctl->nx;
				lay = -a->r*ctl->ny;
				laz = -a->r*ctl->nz;					
				Vax = a->vx + a->wy*laz - a->wz*lay;
				Vay = a->vy + a->wz*lax - a->wx*laz;
				Vaz = a->vz + a->wx*lay - a->wy*lax;
				
				lbx = b->r*ctl->nx;
				lby = b->r*ctl->ny;
				lbz = b->r*ctl->nz;
				Vbx = b->vx + b->wy*lbz - b->wz*lby;
				Vby = b->vy + b->wz*lbx - b->wx*lbz;
				Vbz = b->vz + b->wx*lby - b->wy*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);
								
				/* Cas du contact physique */
				if(ctl->delta <= 0){
					//Calcul de la masse effective
					meff = a->m*b->m/(a->m+b->m);						
					// Coefficient visqueux normale fonction de la masse effective meff, de la raideur k et de la restituion en
					if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);
					// Dynamique
					if(ModelTg == 0){
						// Calcul du coefficient visqueux tangentiel
						gt = 10000;
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						// Recherche de l'ancien xsi
						xsi = a->FoundIt(b->Num(),0,-9);
						// Calcul de N, T et Xsi 
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
						// Enregistrement du nouveau Xsi
						a->AddXsi(xsi,b->Num(),0,-9);
						b->AddXsi(xsi,a->Num(),0,-9);
					}
				}
			       
				// Calcul de la force de contact
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
										
				// Incrementation de la force et du moment sur chaque corps
				a->Fx += Fx;	
				a->Fy += Fy;	
				a->Fz += Fz;
				a->Mx += (lay*Fz-laz*Fy);
				a->My += (laz*Fx-lax*Fz);
				a->Mz += (lax*Fy-lay*Fx);
				
				b->Fx -= Fx;	
				b->Fy -= Fy;	
				b->Fz -= Fz;
				b->Mx -= (lby*Fz-lbz*Fy);
				b->My -= (lbz*Fx-lbx*Fz);
				b->Mz -= (lbx*Fy-lby*Fx);
				
				ctl->xi = a->x;
				ctl->yi = a->y;
				ctl->zi = a->z;
				ctl->xf = b->x;
				ctl->yf = b->y;
				ctl->zf = b->z;
				break;
				
			case 10:
				// Cas du contact entre une sphere et une particules
				a = ctl->sa;
				bb = ctl->bb;
				nb = ctl->nbb;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = -a->r*ctl->nx;
				lay = -a->r*ctl->ny;
				laz = -a->r*ctl->nz;					
				Vax = a->vx + a->wy*laz - a->wz*lay;
				Vay = a->vy + a->wz*lax - a->wx*laz;
				Vaz = a->vz + a->wx*lay - a->wy*lax;
				
				lbx = bb->r[nb]*ctl->nx + (bb->xg[nb] - bb->x);
				lby = bb->r[nb]*ctl->ny + (bb->yg[nb] - bb->y);
				lbz = bb->r[nb]*ctl->nz + (bb->zg[nb] - bb->z);
				Vbx = bb->vx + bb->wy*lbz - bb->wz*lby;
				Vby = bb->vy + bb->wz*lbx - bb->wx*lbz;
				Vbz = bb->vz + bb->wx*lby - bb->wy*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);

				/* Cas du contact physique */
				if(ctl->delta <= 0){
					//Calcul de la masse effective
					meff = a->m*bb->m/(a->m+bb->m);	
					// Coefficient visqueux normale fonction de la masse effective meff, de la raideur k et de la restituion en
					if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);
					// Dynamique
					if(ModelTg == 0){
						// Calcul du coefficient visqueux tangentiel
						gt = 10000;
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						// Recherche de l'ancien xsi
						xsi = bb->FoundIt(a->Num(),10,nb,-9);
						// Calcul de N, T et Xsi 
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
						// Enregistrement du nouveau Xsi
						a->AddXsi(xsi,bb->Num(),10,nb);
						bb->AddXsi(xsi,a->Num(),10,nb,-9);
					}
				}
				
				// Calcul de la force de contact
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

				// Incrementation de la force et du moment sur chaque corps
				a->Fx += Fx;	
				a->Fy += Fy;	
				a->Fz += Fz;
				a->Mx += (lay*Fz-laz*Fy);
				a->My += (laz*Fx-lax*Fz);
				a->Mz += (lax*Fy-lay*Fx);
				
				bb->Fx -= Fx;	
				bb->Fy -= Fy;	
				bb->Fz -= Fz;
				bb->Mx -= (lby*Fz-lbz*Fy);
				bb->My -= (lbz*Fx-lbx*Fz);
				bb->Mz -= (lbx*Fy-lby*Fx);		
				break;
				
			case 20:
				// Cas du contact entre deux particules
				ba = ctl->ba;
				bb = ctl->bb;
				na = ctl->nba;
				nb = ctl->nbb;
								
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = -ba->r[na]*ctl->nx + (ba->xg[na] - ba->x);
				lay = -ba->r[na]*ctl->ny + (ba->yg[na] - ba->y);
				laz = -ba->r[na]*ctl->nz + (ba->zg[na] - ba->z);
				Vax = ba->vx + ba->wy*laz - ba->wz*lay;
				Vay = ba->vy + ba->wz*lax - ba->wx*laz;
				Vaz = ba->vz + ba->wx*lay - ba->wy*lax;
				
				lbx = bb->r[nb]*ctl->nx + (bb->xg[nb] - bb->x);
				lby = bb->r[nb]*ctl->ny + (bb->yg[nb] - bb->y);
				lbz = bb->r[nb]*ctl->nz + (bb->zg[nb] - bb->z);
				Vbx = bb->vx + bb->wy*lbz - bb->wz*lby;
				Vby = bb->vy + bb->wz*lbx - bb->wx*lbz;
				Vbz = bb->vz + bb->wx*lby - bb->wy*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);

								
				/* Cas du contact physique */
				if(ctl->delta <= 0){
					meff = ba->m*bb->m/(ba->m+bb->m);					
					if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);
					// Dynamique
					if(ModelTg == 0){
						gt = 10000;				
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						xsi = ba->FoundIt(bb->Num(),20,na,nb);
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
						ba->AddXsi(xsi,bb->Num(),20,na,nb);
						bb->AddXsi(xsi,ba->Num(),20,nb,na);
					}
				}
				
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
				
				ba->Fx += Fx;	
				ba->Fy += Fy;	
				ba->Fz += Fz;
				ba->Mx += (lay*Fz-laz*Fy);
				ba->My += (laz*Fx-lax*Fz);
				ba->Mz += (lax*Fy-lay*Fx);		
				
				bb->Fx -= Fx;	
				bb->Fy -= Fy;	
				bb->Fz -= Fz;
				bb->Mx -= (lby*Fz-lbz*Fy);
				bb->My -= (lbz*Fx-lbx*Fz);
				bb->Mz -= (lbx*Fy-lby*Fx);		
				break;								
			case 1:			
				// Cas du contact entre une sphere et un plan
				a = ctl->sa;
				p = ctl->pa;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = a->r*ctl->nx;
				lay = a->r*ctl->ny;
				laz = a->r*ctl->nz;					
				Vax = a->vx + a->wy*laz - a->wz*lay;
				Vay = a->vy + a->wz*lax - a->wx*laz;
				Vaz = a->vz + a->wx*lay - a->wy*lax;
				
				lbx = ctl->px - p->V.ox;
				lby = ctl->py - p->V.oy;
				lbz = ctl->pz - p->V.oz;
                
				wbx = p->V.ValueOfWx(TIME);
				wby = p->V.ValueOfWy(TIME);
				wbz = p->V.ValueOfWz(TIME);						
				Vbx = p->V.ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = p->V.ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = p->V.ValueOfVz(TIME) + wbx*lby-wby*lbx;

                /*
                p->affiche();
                printf("b = (%e,%e,%e)\n",a->x,a->y,a->z);
                printf("n = (%e,%e,%e)\n",ctl->nx,ctl->ny,ctl->nz);
                //printf("np = (%e,%e,%e)\n",p->nx,p->ny,p->nz);
                //printf("q = (%e,%e,%e,%e)\n",p->q0,p->q1,p->q2,p->q3);
                printf("p = (%e,%e,%e)\n",ctl->px,ctl->py,ctl->pz);
                printf("la = (%e,%e,%e)\n",lax,lay,laz);
                printf("lb = (%e,%e,%e)\n",lbx,lby,lbz);
                */
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
				

				/* Cas du contact physique */
				if(ctl->delta <= 0){
					meff = a->m;				
					if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);
					
					// Dynamique
					if(ModelTg == 0){
						gt = 10000;
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						xsi = a->FoundIt(p->numero,1,-9);		
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);			
						a->AddXsi(xsi,p->numero,1,-9);
					}
				}
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);				
                /*
                printf("F = (%e,%e,%e)\n\n",Fx,Fy,Fz);
                
                if(Fx != 0)
                    exit(0);
                if(p->numero == 1)
                    exit(0);
                 */
                
				// Ajout de la force et du moment				
				a->Fx -= Fx;	
				a->Fy -= Fy;	
				a->Fz -= Fz;
				a->Mx -= (lay*Fz-laz*Fy);
				a->My -= (laz*Fx-lax*Fz);
				a->Mz -= (lax*Fy-lay*Fx);			
				
				
				p->Fx += Fx;
				p->Fy += Fy;
				p->Fz += Fz;	
				p->Mx += (lby*Fz-lbz*Fy);
				p->My += (lbz*Fx-lbx*Fz);
				p->Mz += (lbx*Fy-lby*Fx);		
				
				ctl->xi = a->x;
				ctl->yi = a->y;
				ctl->zi = a->z;
				ctl->xf = a->x + lax;
				ctl->yf = a->y + lay;
				ctl->zf = a->z + laz;

				break;
			case 11:
				// Cas du contact entre une particule et un plan
				ba = ctl->ba;
				na = ctl->nba;
				p = ctl->pa;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = ba->r[na]*ctl->nx + (ba->xg[na] - ba->x);
				lay = ba->r[na]*ctl->ny + (ba->yg[na] - ba->y);
				laz = ba->r[na]*ctl->nz + (ba->zg[na] - ba->z);				
				Vax = ba->vx + ba->wy*laz - ba->wz*lay;
				Vay = ba->vy + ba->wz*lax - ba->wx*laz;
				Vaz = ba->vz + ba->wx*lay - ba->wy*lax;
				
				lbx = ctl->px - p->V.ox;
				lby = ctl->py - p->V.oy;
				lbz = ctl->pz - p->V.oz;				
				wbx = p->V.ValueOfWx(TIME);
				wby = p->V.ValueOfWy(TIME);
				wbz = p->V.ValueOfWz(TIME);						
				Vbx = p->V.ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = p->V.ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = p->V.ValueOfVz(TIME) + wbx*lby-wby*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
		
				/* Cas du contact physique */
				if(ctl->delta <= 0){
					meff = ba->m;				
					if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);
				
					// Dynamique
					if(ModelTg == 0){
						gt = 10000;
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						xsi = ba->FoundIt(p->numero,11,na,-9);	
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);		
						ba->AddXsi(xsi,p->numero,11,na,-9);
					}
				}
								
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

				// Ajout de la force et du moment						
				ba->Fx -= Fx;	
				ba->Fy -= Fy;	
				ba->Fz -= Fz;
				ba->Mx -= (lay*Fz-laz*Fy);
				ba->My -= (laz*Fx-lax*Fz);
				ba->Mz -= (lax*Fy-lay*Fx);	
				
				p->Fx += Fx;
				p->Fy += Fy;
				p->Fz += Fz;	
				p->Mx += (lby*Fz-lbz*Fy);
				p->My += (lbz*Fx-lbx*Fz);
				p->Mz += (lbx*Fy-lby*Fx);		
				break;
			case 2:
				// Cas du contact entre une sphere et un disque
				a = ctl->sa;
				pr = ctl->par;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = a->r*ctl->nx;
				lay = a->r*ctl->ny;
				laz = a->r*ctl->nz;					
				Vax = a->vx + a->wy*laz - a->wz*lay;
				Vay = a->vy + a->wz*lax - a->wx*laz;
				Vaz = a->vz + a->wx*lay - a->wy*lax;
				
				lbx = ctl->px - pr->V.ox;
				lby = ctl->py - pr->V.oy;
				lbz = ctl->pz - pr->V.oz;				
				wbx = pr->V.ValueOfWx(TIME);
				wby = pr->V.ValueOfWy(TIME);
				wbz = pr->V.ValueOfWz(TIME);						
				Vbx = pr->V.ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = pr->V.ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = pr->V.ValueOfVz(TIME) + wbx*lby-wby*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
				
				/* Cas du contact physique */
				if(ctl->delta < 0){
					meff = a->m;				
					if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);

					// Dynamique
					if(ModelTg == 0){
						gt = 10000;
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						xsi = a->FoundIt(pr->numero,2,-9);	
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);			
						a->AddXsi(xsi,pr->numero,2,-9);
					}					
				}
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

				// Ajout de la force et du moment
				a->Fx -= Fx;	
				a->Fy -= Fy;	
				a->Fz -= Fz;
				a->Mx -= (lay*Fz-laz*Fy);
				a->My -= (laz*Fx-lax*Fz);
				a->Mz -= (lax*Fy-lay*Fx);			
				
				pr->Fx += Fx;
				pr->Fy += Fy;
				pr->Fz += Fz;
				pr->Mx += (lby*Fz-lbz*Fy);
				pr->My += (lbz*Fx-lbx*Fz);
				pr->Mz += (lbx*Fy-lby*Fx);	
				
				ctl->xi = a->x;
				ctl->yi = a->y;
				ctl->zi = a->z;
				ctl->xf = a->x + lax;
				ctl->yf = a->y + lay;
				ctl->zf = a->z + laz;

				break;
	
			case 12:
				// Cas du contact entre une particule et un disque
				ba = ctl->ba;
				na = ctl->nba;
				pr = ctl->par;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = ba->r[na]*ctl->nx + (ba->xg[na] - ba->x);
				lay = ba->r[na]*ctl->ny + (ba->yg[na] - ba->y);
				laz = ba->r[na]*ctl->nz + (ba->zg[na] - ba->z);				
				Vax = ba->vx + ba->wy*laz - ba->wz*lay;
				Vay = ba->vy + ba->wz*lax - ba->wx*laz;
				Vaz = ba->vz + ba->wx*lay - ba->wy*lax;
								
				lbx = ctl->px - pr->V.ox;
				lby = ctl->py - pr->V.oy;
				lbz = ctl->pz - pr->V.oz;				
				wbx = pr->V.ValueOfWx(TIME);
				wby = pr->V.ValueOfWy(TIME);
				wbz = pr->V.ValueOfWz(TIME);						
				Vbx = pr->V.ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = pr->V.ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = pr->V.ValueOfVz(TIME) + wbx*lby-wby*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
				
				/* Cas du contact physique */
				if(ctl->delta < 0){
					meff = ba->m;				
					if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);
										
					// Dynamique
					if(ModelTg == 0){
						gt = 10000;
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						xsi = ba->FoundIt(pr->numero,12,na,-9);	
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
						ba->AddXsi(xsi,pr->numero,12,na,-9);
					}					
				}
				
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

				// Ajout de la force et du moment
				ba->Fx -= Fx;	
				ba->Fy -= Fy;	
				ba->Fz -= Fz;
				ba->Mx -= (lay*Fz-laz*Fy);
				ba->My -= (laz*Fx-lax*Fz);
				ba->Mz -= (lax*Fy-lay*Fx);		
				
				pr->Fx += Fx;
				pr->Fy += Fy;
				pr->Fz += Fz;
				pr->Mx += (lby*Fz-lbz*Fy);
				pr->My += (lbz*Fx-lbx*Fz);
				pr->Mz += (lbx*Fy-lby*Fx);	
				break;				

			case 3:
				// Cas du contact entre une sphere et un cone
				a = ctl->sa;
				cne = ctl->cn;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = a->r*ctl->nx;
				lay = a->r*ctl->ny;
				laz = a->r*ctl->nz;					
				Vax = a->vx + a->wy*laz - a->wz*lay;
				Vay = a->vy + a->wz*lax - a->wx*laz;
				Vaz = a->vz + a->wx*lay - a->wy*lax;
				
				lbx = ctl->px - cne->V.ox;
				lby = ctl->py - cne->V.oy;
				lbz = ctl->pz - cne->V.oz;				
				wbx = cne->V.ValueOfWx(TIME);
				wby = cne->V.ValueOfWy(TIME);
				wbz = cne->V.ValueOfWz(TIME);						
				Vbx = cne->V.ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = cne->V.ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = cne->V.ValueOfVz(TIME) + wbx*lby-wby*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
							
				meff = a->m;
				if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);
				
				// Dynamique
				if(ModelTg == 0){
					gt = 10000;
					// Calcul de N et T
					computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
				}
				// Statique
				else{
					N = k*(-ctl->delta)-g0*Vn;
					if(N < 0) N = 0;
					xsi = a->FoundIt(cne->numero,3,-9);					
					computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
					a->AddXsi(xsi,cne->numero,3,-9);
				}			
							
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
				
				// Ajout de la force et du moment
				a->Fx -= Fx;	
				a->Fy -= Fy;	
				a->Fz -= Fz;
				a->Mx -= (lay*Fz-laz*Fy);
				a->My -= (laz*Fx-lax*Fz);
				a->Mz -= (lax*Fy-lay*Fx);
                
                cne->Fx += Fx;
				cne->Fy += Fy;
				cne->Fz += Fz;
				cne->Mx += (lay*Fz-laz*Fy);
				cne->My += (laz*Fx-lax*Fz);
				cne->Mz += (lax*Fy-lay*Fx);

				ctl->xi = a->x;
				ctl->yi = a->y;
				ctl->zi = a->z;
				ctl->xf = a->x + lax;
				ctl->yf = a->y + lay;
				ctl->zf = a->z + laz;

				break;
			case 13:
				// Cas du contact entre une particule et un cone
				ba = ctl->ba;
				na = ctl->nba;
				cne = ctl->cn;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = ba->r[na]*ctl->nx + (ba->xg[na] - ba->x);
				lay = ba->r[na]*ctl->ny + (ba->yg[na] - ba->y);
				laz = ba->r[na]*ctl->nz + (ba->zg[na] - ba->z);				
				Vax = ba->vx + ba->wy*laz - ba->wz*lay;
				Vay = ba->vy + ba->wz*lax - ba->wx*laz;
				Vaz = ba->vz + ba->wx*lay - ba->wy*lax;
				
				lbx = ctl->px - cne->V.ox;
				lby = ctl->py - cne->V.oy;
				lbz = ctl->pz - cne->V.oz;				
				wbx = cne->V.ValueOfWx(TIME);
				wby = cne->V.ValueOfWy(TIME);
				wbz = cne->V.ValueOfWz(TIME);						
				Vbx = cne->V.ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = cne->V.ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = cne->V.ValueOfVz(TIME) + wbx*lby-wby*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
				
				meff = ba->m;
				// Parametre du contact fonction de plan et de l'espece de grain en contact
				if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);
				
				// Dynamique
				if(ModelTg == 0){
					gt = 10000;
					// Calcul de N et T
					computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
				}
				// Statique
				else{
					// Calcul de la composante normale de la force de contact
					N = k*(-ctl->delta)-g0*Vn;
					if(N < 0) N = 0;

					xsi = ba->FoundIt(cne->numero,13,na,-9);					
					computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
					ba->AddXsi(xsi,cne->numero,13,na,-9);
				}
				
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
				
				// Ajout de la force et du moment
				ba->Fx -= Fx;	
				ba->Fy -= Fy;	
				ba->Fz -= Fz;
				ba->Mx -= (lay*Fz-laz*Fy);
				ba->My -= (laz*Fx-lax*Fz);
				ba->Mz -= (lax*Fy-lay*Fx);
                cne->Fx += Fx;
				cne->Fy += Fy;
				cne->Fz += Fz;
				cne->Mx += (lay*Fz-laz*Fy);
				cne->My += (laz*Fx-lax*Fz);
				cne->Mz += (lax*Fy-lay*Fx);

				break;
			case 4:
				// Cas du contact entre une sphere et un coude
				a = ctl->sa;
				elw = ctl->ew;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = -a->r*ctl->nx;
				lay = -a->r*ctl->ny;
				laz = -a->r*ctl->nz;					
				Vax = a->vx + a->wy*laz - a->wz*lay;
				Vay = a->vy + a->wz*lax - a->wx*laz;
				Vaz = a->vz + a->wx*lay - a->wy*lax;
				
				lbx = ctl->px - elw->V.ox;
				lby = ctl->py - elw->V.oy;
				lbz = ctl->pz - elw->V.oz;				
				wbx = elw->V.ValueOfWx(TIME);
				wby = elw->V.ValueOfWy(TIME);
				wbz = elw->V.ValueOfWz(TIME);						
				Vbx = elw->V.ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = elw->V.ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = elw->V.ValueOfVz(TIME) + wbx*lby-wby*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
				
				meff = a->m;
				if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);
			
				// Dynamique
				if(ModelTg == 0){
					gt = 10000;
					// Calcul de N et T
					computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
				}
				// Statique
				else{
					xsi = a->FoundIt(elw->numero,4,-9);
					computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
					a->AddXsi(xsi,elw->numero,4,-9);
				}		
				
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

				// Ajout de la force et du moment
				a->Fx -= Fx;	
				a->Fy -= Fy;	
				a->Fz -= Fz;
				a->Mx -= (lay*Fz-laz*Fy);
				a->My -= (laz*Fx-lax*Fz);
				a->Mz -= (lax*Fy-lay*Fx);			
			
				break;
			case 14 :
				// Cas du contact entre une particule et un coude
				ba = ctl->ba;
				na = ctl->nba;
				elw = ctl->ew;

				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = -ba->r[na]*ctl->nx + (ba->xg[na] - ba->x);
				lay = -ba->r[na]*ctl->ny + (ba->yg[na] - ba->y);
				laz = -ba->r[na]*ctl->nz + (ba->zg[na] - ba->z);
				Vax = ba->vx + ba->wy*laz - ba->wz*lay;
				Vay = ba->vy + ba->wz*lax - ba->wx*laz;
				Vaz = ba->vz + ba->wx*lay - ba->wy*lax;
				
				lbx = ctl->px - elw->V.ox;
				lby = ctl->py - elw->V.oy;
				lbz = ctl->pz - elw->V.oz;				
				wbx = elw->V.ValueOfWx(TIME);
				wby = elw->V.ValueOfWy(TIME);
				wbz = elw->V.ValueOfWz(TIME);						
				Vbx = elw->V.ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = elw->V.ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = elw->V.ValueOfVz(TIME) + wbx*lby-wby*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
							
				meff = ba->m;
				if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);
							
				// Dynamique
				if(ModelTg == 0){
					gt = 10000;
					// Calcul de N et T
					computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
				}
				// Statique
				else{
					xsi = ba->FoundIt(elw->numero,14,na,-9);
					computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
					ba->AddXsi(xsi,elw->numero,4,na,-9);
				}					
				
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

				// Ajout de la force et du moment
				ba->Fx -= Fx;	
				ba->Fy -= Fy;	
				ba->Fz -= Fz;
				ba->Mx -= (lay*Fz-laz*Fy);
				ba->My -= (laz*Fx-lax*Fz);
				ba->Mz -= (lax*Fy-lay*Fx);		
				
				break;
            case 5:
                // Cas du contact entre deux spheres
				a = ctl->sa;
				b = ctl->sb;
			    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = -a->r*ctl->nx;
				lay = -a->r*ctl->ny;
				laz = -a->r*ctl->nz;
				Vax = a->vx + a->wy*laz - a->wz*lay;
				Vay = a->vy + a->wz*lax - a->wx*laz;
				Vaz = a->vz + a->wx*lay - a->wy*lax;
				
				lbx = -b->r*ctl->nx;
				lby = -b->r*ctl->ny;
				lbz = -b->r*ctl->nz;
				Vbx = b->vx + b->wy*lbz - b->wz*lby;
				Vby = b->vy + b->wz*lbx - b->wx*lbz;
				Vbz = b->vz + b->wx*lby - b->wy*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);
                
				/* Cas du contact physique */
				if(ctl->delta <= 0){
					//Calcul de la masse effective
					meff = a->m*b->m/(a->m+b->m);
					// Coefficient visqueux normale fonction de la masse effective meff, de la raideur k et de la restituion en
					if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);
					// Dynamique
					if(ModelTg == 0){
						// Calcul du coefficient visqueux tangentiel
						gt = 10000;
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						// Recherche de l'ancien xsi
						xsi = a->FoundIt(b->Num(),0,-9);
						// Calcul de N, T et Xsi
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
						// Enregistrement du nouveau Xsi
						a->AddXsi(xsi,b->Num(),0,-9);
						b->AddXsi(xsi,a->Num(),0,-9);
					}
				}
				// Calcul de la force de contact
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
                
				// Incrementation de la force et du moment sur chaque corps
				a->Fx += Fx;
				a->Fy += Fy;
				a->Fz += Fz;
				a->Mx += (lay*Fz-laz*Fy);
				a->My += (laz*Fx-lax*Fz);
				a->Mz += (lax*Fy-lay*Fx);
				
				b->Fx -= Fx;
				b->Fy -= Fy;
				b->Fz -= Fz;
				b->Mx -= (lby*Fz-lbz*Fy);
				b->My -= (lbz*Fx-lbx*Fz);
				b->Mz -= (lbx*Fy-lby*Fx);
                break;
            case 6:
                // Cas du contact entre une sphere et une particules
				a = ctl->sa;
				bb = ctl->bb;
				nb = ctl->nbb;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lbx = -a->r*ctl->nx;
				lby = -a->r*ctl->ny;
				lbz = -a->r*ctl->nz;
				Vbx = a->vx + a->wy*lbz - a->wz*lby;
				Vby = a->vy + a->wz*lbx - a->wx*lbz;
				Vbz = a->vz + a->wx*lby - a->wy*lbx;
            
				lax = -bb->r[nb]*ctl->nx + (bb->xg[nb] - bb->x);
				lay = -bb->r[nb]*ctl->ny + (bb->yg[nb] - bb->y);
				laz = -bb->r[nb]*ctl->nz + (bb->zg[nb] - bb->z);
				Vax = bb->vx + bb->wy*laz - bb->wz*lay;
				Vay = bb->vy + bb->wz*lax - bb->wx*laz;
				Vaz = bb->vz + bb->wx*lay - bb->wy*lax;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);
                
				/* Cas du contact physique */
				if(ctl->delta <= 0){
					//Calcul de la masse effective
					meff = a->m*bb->m/(a->m+bb->m);
					// Coefficient visqueux normale fonction de la masse effective meff, de la raideur k et de la restituion en
					if(en != 0)
						g0 = sqrt(4*meff*k/(1+(M_PI/log(en))*(M_PI/log(en))));
					else
						g0 = sqrt(4*meff*k);
					// Dynamique
					if(ModelTg == 0){
						// Calcul du coefficient visqueux tangentiel
						gt = 10000;
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						// Recherche de l'ancien xsi
						xsi = bb->FoundIt(a->Num(),10,nb,-9);
						// Calcul de N, T et Xsi
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
						// Enregistrement du nouveau Xsi
						a->AddXsi(xsi,bb->Num(),10,nb);
						bb->AddXsi(xsi,a->Num(),10,nb,-9);
					}
				}
				
				// Calcul de la force de contact
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
                
				// Incrementation de la force et du moment sur chaque corps
				a->Fx -= Fx;
				a->Fy -= Fy;
				a->Fz -= Fz;
				a->Mx -= (lby*Fz-lbz*Fy);
				a->My -= (lbz*Fx-lbx*Fz);
				a->Mz -= (lbx*Fy-lby*Fx);
				
				bb->Fx += Fx;
				bb->Fy += Fy;
				bb->Fz += Fz;
				bb->Mx += (lay*Fz-laz*Fy);
				bb->My += (laz*Fx-lax*Fz);
				bb->Mz += (lax*Fy-lay*Fx);
                break;
		}
		ctl->Fx = Fx;
		ctl->Fy = Fy;
		ctl->Fz = Fz;
		ctl++;
	}
}

