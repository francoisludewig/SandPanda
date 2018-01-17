#include "../Includes/ComputingForce.h"

#include "../Includes/Velocity.h"
#include "../Includes/Gravity.h"
#include "../Includes/Plan.h"
#include "../Includes/PlanR.h"
#include "../Includes/Cone.h"
#include "../Includes/Elbow.h"
#include "../Includes/Sphere.h"
#include "../Includes/Body.h"
#include "../Includes/Contact.h"
#include "../Includes/ReadWrite.h"
#include "../Includes/ContactDetection.h"
#include "../Includes/Elongation.h"
#include "../Includes/Data.h"

/* Fonction qui initialise tous les obejts avant chaque etape de calcul */
void ComputeForce::InitForTimeStep(const int & Nct, std::vector<Sphere> & sph, std::vector<Body> & bd, Contact *ct, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb) noexcept {
	for(auto& sphere : sph)
		sphere.initTimeStep();
	for(auto& body : bd)
		body.TimeStepInitialization();
	for(auto& plan : pl)
		plan.TimeStepInitialization();
	for(auto& disk : plr)
		disk.TimeStepInitialization();
	for(auto& cone : co)
		cone.TimeStepInitialization();

	for(int i = 0 ; i < Nct ; ++i)
		ct[i].TimeStepInitialization();
}

/* Fonction qui initialise tous les obejts avant chaque etape de calcul en version parallel OMP */
void ComputeForce::InitForTimeStepOMP(const int & Nsph, const int & Nbd , const int & Nct , const int & Ncta , const int & Nctb , const int & Nctc ,int & Npl, int & Nplr, int & Nco, int & Nelb, std::vector<Sphere> & sph, std::vector<Body> & bd, Contact *ct, Contact *cta, Contact *ctb, Contact *ctc, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb) noexcept {
	// Sphere
	for(int i = 0 ; i < Nsph ; i++)
		sph[i].initTimeStep();
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
		pl[i].TimeStepInitialization();
	// Disque
	for(int i = 0 ; i < Nplr ; i++)
		plr[i].TimeStepInitialization();
    // Cone
	for(int i = 0 ; i < Nco ; i++)
		co[i].TimeStepInitialization();

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
			case Contact::Type::None:
				break;
			case Contact::Type::SphereSphere: //0
				// Cas du contact entre deux spheres
				a = ctl->sa;
				b = ctl->sb;
			    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = -a->Radius()*ctl->nx;
				lay = -a->Radius()*ctl->ny;
				laz = -a->Radius()*ctl->nz;
				a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

				lbx = b->Radius()*ctl->nx;
				lby = b->Radius()*ctl->ny;
				lbz = b->Radius()*ctl->nz;
				b->PointVelocity(Vbx, Vby, Vbz, lbx, lby, lbz);

				// Calcul des vitesses local du contact
				computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);
								
				/* Cas du contact physique */
				if(ctl->delta <= 0){
					//Calcul de la masse effective
					meff = a->Mass()*b->Mass()/(a->Mass()+b->Mass());
					// Coefficient visqueux normale fonction de la masse effective meff, de la raideur k et de la restituion en
					g0 = DampingCoefficient(en, meff, k);
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
						xsi = a->GetElongationManager().GetElongation(b->Num(),Contact::Type::SphereSphere,-9);

						// Calcul de N, T et Xsi 
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
						// Enregistrement du nouveau Xsi


						//printf("Add Xsi = %e\t%e\t%e\n\n", xsi.x, xsi.y, xsi.z);
						a->GetElongationManager().AddElongation(xsi,b->Num(),Contact::Type::SphereSphere,-9);
						b->GetElongationManager().AddElongation(xsi,a->Num(),Contact::Type::SphereSphere,-9);
					}
				}
			       
				// Calcul de la force de contact
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
										
				// Incrementation de la force et du moment sur chaque corps
				a->AddForce(Fx, Fy, Fz);
				a->AddMomemtum((lay*Fz-laz*Fy), (laz*Fx-lax*Fz), (lax*Fy-lay*Fx));

				b->AddForce(-Fx, -Fy, -Fz);
				b->AddMomemtum(-(lby*Fz-lbz*Fy), -(lbz*Fx-lbx*Fz), -(lbx*Fy-lby*Fx));

				ctl->xi = a->X();
				ctl->yi = a->Y();
				ctl->zi = a->Z();
				ctl->xf = b->X();
				ctl->yf = b->Y();
				ctl->zf = b->Z();
				break;
				
			case Contact::Type::SphereBody: //10
				// Cas du contact entre une sphere et une particules
				a = ctl->sa;
				bb = ctl->bb;
				nb = ctl->nbb;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = -a->Radius()*ctl->nx;
				lay = -a->Radius()*ctl->ny;
				laz = -a->Radius()*ctl->nz;
				a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
				std::tie(lbx, lby, lbz) = bb->Lever(ctl->nx, ctl->ny, ctl->nz, nb);
				bb->PointVelocity(Vbx, Vby, Vbz, lbx, lby, lbz);
				
				// Calcul des vitesses local du contact
				computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);

				/* Cas du contact physique */
				if(ctl->delta <= 0){
					//Calcul de la masse effective
					meff = a->Mass()*bb->Mass()/(a->Mass()+bb->Mass());
					// Coefficient visqueux normale fonction de la masse effective meff, de la raideur k et de la restituion en
					g0 = DampingCoefficient(en, meff, k);
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
						xsi = bb->GetElongationManager().GetElongation(a->Num(),Contact::Type::SphereBody,nb,-9);
						// Calcul de N, T et Xsi 
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
						// Enregistrement du nouveau Xsi
						a->GetElongationManager().AddElongation(xsi,bb->Num(),Contact::Type::SphereBody,nb);
						bb->GetElongationManager().AddElongation(xsi,a->Num(),Contact::Type::SphereBody,nb,-9);
					}
				}
				
				// Calcul de la force de contact
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

				// Incrementation de la force et du moment sur chaque corps
				a->AddForce(Fx, Fy, Fz);
				a->AddMomemtum((lay*Fz-laz*Fy), (laz*Fx-lax*Fz), (lax*Fy-lay*Fx));

				bb->AddForce(-Fx, -Fy, -Fz);
				bb->AddMomemtum(-(lby*Fz-lbz*Fy), -(lbz*Fx-lbx*Fz), -(lbx*Fy-lby*Fx));
				break;
				
			case Contact::Type::BodyBody: //20
				// Cas du contact entre deux particules
				ba = ctl->ba;
				bb = ctl->bb;
				na = ctl->nba;
				nb = ctl->nbb;
								
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				std::tie(lax, lay, laz) = ba->Lever(-ctl->nx, -ctl->ny, -ctl->nz, na);
				ba->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
				
				std::tie(lbx, lby, lbz) = bb->Lever( ctl->nx,  ctl->ny,  ctl->nz, nb);
				bb->PointVelocity(Vbx, Vby, Vbz, lbx, lby, lbz);
				
				// Calcul des vitesses local du contact
				computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);
				
				/* Cas du contact physique */
				if(ctl->delta <= 0){
					meff = ba->Mass()*bb->Mass()/(ba->Mass()+bb->Mass());
					g0 = DampingCoefficient(en, meff, k);
					// Dynamique
					if(ModelTg == 0){
						gt = 10000;				
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						xsi = ba->GetElongationManager().GetElongation(bb->Num(),Contact::Type::BodyBody,na,nb);
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
						ba->GetElongationManager().AddElongation(xsi,bb->Num(),Contact::Type::BodyBody,na,nb);
						bb->GetElongationManager().AddElongation(xsi,ba->Num(),Contact::Type::BodyBody,nb,na);
					}
				}
				
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
				
				ba->AddForce(Fx, Fy, Fz);
				ba->AddMomemtum((lay*Fz-laz*Fy), (laz*Fx-lax*Fz), (lax*Fy-lay*Fx));

				bb->AddForce(-Fx, -Fy, -Fz);
				bb->AddMomemtum(-(lby*Fz-lbz*Fy), -(lbz*Fx-lbx*Fz), -(lbx*Fy-lby*Fx));
				break;								
			case Contact::Type::SpherePlan: //1
				// Cas du contact entre une sphere et un plan
				a = ctl->sa;
				p = ctl->pa;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = a->Radius()*ctl->nx;
				lay = a->Radius()*ctl->ny;
				laz = a->Radius()*ctl->nz;
				a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
				
				lbx = ctl->px - p->GetV().ox;
				lby = ctl->py - p->GetV().oy;
				lbz = ctl->pz - p->GetV().oz;
                
				wbx = p->GetV().ValueOfWx(TIME);
				wby = p->GetV().ValueOfWy(TIME);
				wbz = p->GetV().ValueOfWz(TIME);
				Vbx = p->GetV().ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = p->GetV().ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = p->GetV().ValueOfVz(TIME) + wbx*lby-wby*lbx;

				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
				

				/* Cas du contact physique */
				if(ctl->delta <= 0){
					meff = a->Mass();
					g0 = DampingCoefficient(en, meff, k);
					
					// Dynamique
					if(ModelTg == 0){
						gt = 10000;
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						xsi = a->GetElongationManager().GetElongation(p->Numero(),Contact::Type::SpherePlan,-9);
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);			
						a->GetElongationManager().AddElongation(xsi,p->Numero(),Contact::Type::SpherePlan,-9);
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
				a->AddForce(-Fx, -Fy, -Fz);
				a->AddMomemtum(-(lay*Fz-laz*Fy), -(laz*Fx-lax*Fz), -(lax*Fy-lay*Fx));

			    p->AddForce(Fx, Fy, Fz);
			    p->AddMomemtum((lby*Fz-lbz*Fy), (lbz*Fx-lbx*Fz), (lbx*Fy-lby*Fx));
				
				ctl->xi = a->X();
				ctl->yi = a->Y();
				ctl->zi = a->Z();
				ctl->xf = a->X() + lax;
				ctl->yf = a->Y() + lay;
				ctl->zf = a->Z() + laz;

				break;
			case Contact::Type::BodyPlan: //11
				// Cas du contact entre une particule et un plan
				ba = ctl->ba;
				na = ctl->nba;
				p = ctl->pa;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = ba->SphereRadius(na)*ctl->nx + (ba->SphereX(na) - ba->X());
				lay = ba->SphereRadius(na)*ctl->ny + (ba->SphereY(na) - ba->Y());
				laz = ba->SphereRadius(na)*ctl->nz + (ba->SphereZ(na) - ba->Z());
				ba->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
				
				lbx = ctl->px - p->GetV().ox;
				lby = ctl->py - p->GetV().oy;
				lbz = ctl->pz - p->GetV().oz;
				wbx = p->GetV().ValueOfWx(TIME);
				wby = p->GetV().ValueOfWy(TIME);
				wbz = p->GetV().ValueOfWz(TIME);
				Vbx = p->GetV().ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = p->GetV().ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = p->GetV().ValueOfVz(TIME) + wbx*lby-wby*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
		
				/* Cas du contact physique */
				if(ctl->delta <= 0){
					meff = ba->Mass();
					g0 = DampingCoefficient(en, meff, k);
					// Dynamique
					if(ModelTg == 0){
						gt = 10000;
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						xsi = ba->GetElongationManager().GetElongation(p->Numero(),Contact::Type::BodyPlan,na,-9);
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);		
						ba->GetElongationManager().AddElongation(xsi,p->Numero(),Contact::Type::BodyPlan,na,-9);
					}
				}
								
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

				// Ajout de la force et du moment

				ba->AddForce(-Fx, -Fy, -Fz);
				ba->AddMomemtum(-(lay*Fz-laz*Fy), -(laz*Fx-lax*Fz), -(lax*Fy-lay*Fx));

				p->AddForce(Fx, Fy, Fz);
				p->AddMomemtum((lby*Fz-lbz*Fy), (lbz*Fx-lbx*Fz), (lbx*Fy-lby*Fx));

				break;
			case Contact::Type::SpherePlanR: //2
				// Cas du contact entre une sphere et un disque
				a = ctl->sa;
				pr = ctl->par;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = a->Radius()*ctl->nx;
				lay = a->Radius()*ctl->ny;
				laz = a->Radius()*ctl->nz;
				a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
				
				lbx = ctl->px - pr->GetV().ox;
				lby = ctl->py - pr->GetV().oy;
				lbz = ctl->pz - pr->GetV().oz;
				wbx = pr->GetV().ValueOfWx(TIME);
				wby = pr->GetV().ValueOfWy(TIME);
				wbz = pr->GetV().ValueOfWz(TIME);
				Vbx = pr->GetV().ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = pr->GetV().ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = pr->GetV().ValueOfVz(TIME) + wbx*lby-wby*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
				
				/* Cas du contact physique */
				if(ctl->delta < 0){
					meff = a->Mass();
					g0 = DampingCoefficient(en, meff, k);

					// Dynamique
					if(ModelTg == 0){
						gt = 10000;
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						xsi = a->GetElongationManager().GetElongation(pr->Numero(),Contact::Type::SpherePlanR,-9);
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);			
						a->GetElongationManager().AddElongation(xsi,pr->Numero(),Contact::Type::SpherePlanR,-9);
					}					
				}
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

				// Ajout de la force et du moment
				a->AddForce(-Fx, -Fy, -Fz);
				a->AddMomemtum(-(lay*Fz-laz*Fy), -(laz*Fx-lax*Fz), -(lax*Fy-lay*Fx));

				pr->AddForce(Fx, Fy, Fz);
				pr->AddMomemtum((lby*Fz-lbz*Fy), (lbz*Fx-lbx*Fz), (lbx*Fy-lby*Fx));

				ctl->xi = a->X();
				ctl->yi = a->Y();
				ctl->zi = a->Z();
				ctl->xf = a->X() + lax;
				ctl->yf = a->Y() + lay;
				ctl->zf = a->Z() + laz;

				break;
	
			case Contact::Type::BodyPlanR: //12
				// Cas du contact entre une particule et un disque
				ba = ctl->ba;
				na = ctl->nba;
				pr = ctl->par;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = ba->SphereRadius(na)*ctl->nx + (ba->SphereX(na) - ba->X());
				lay = ba->SphereRadius(na)*ctl->ny + (ba->SphereY(na) - ba->Y());
				laz = ba->SphereRadius(na)*ctl->nz + (ba->SphereZ(na) - ba->Z());
				ba->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
								
				lbx = ctl->px - pr->GetV().ox;
				lby = ctl->py - pr->GetV().oy;
				lbz = ctl->pz - pr->GetV().oz;
				wbx = pr->GetV().ValueOfWx(TIME);
				wby = pr->GetV().ValueOfWy(TIME);
				wbz = pr->GetV().ValueOfWz(TIME);
				Vbx = pr->GetV().ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = pr->GetV().ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = pr->GetV().ValueOfVz(TIME) + wbx*lby-wby*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
				
				/* Cas du contact physique */
				if(ctl->delta < 0){
					meff = ba->Mass();
					g0 = DampingCoefficient(en, meff, k);
										
					// Dynamique
					if(ModelTg == 0){
						gt = 10000;
						// Calcul de N et T
						computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
					}
					// Statique
					else{
						xsi = ba->GetElongationManager().GetElongation(pr->Numero(),Contact::Type::BodyPlanR,na,-9);
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
						ba->GetElongationManager().AddElongation(xsi,pr->Numero(),Contact::Type::BodyPlanR,na,-9);
					}					
				}
				
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

				// Ajout de la force et du moment
				ba->AddForce(-Fx, -Fy, -Fz);
				ba->AddMomemtum(-(lay*Fz-laz*Fy), -(laz*Fx-lax*Fz), -(lax*Fy-lay*Fx));

				pr->AddForce(Fx, Fy, Fz);
				pr->AddMomemtum((lby*Fz-lbz*Fy), (lbz*Fx-lbx*Fz), (lbx*Fy-lby*Fx));
				
				break;				

			case Contact::Type::SphereCone: //3
				// Cas du contact entre une sphere et un cone
				a = ctl->sa;
				cne = ctl->cn;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = a->Radius()*ctl->nx;
				lay = a->Radius()*ctl->ny;
				laz = a->Radius()*ctl->nz;
				a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
				
				lbx = ctl->px - cne->GetV().ox;
				lby = ctl->py - cne->GetV().oy;
				lbz = ctl->pz - cne->GetV().oz;
				wbx = cne->GetV().ValueOfWx(TIME);
				wby = cne->GetV().ValueOfWy(TIME);
				wbz = cne->GetV().ValueOfWz(TIME);
				Vbx = cne->GetV().ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = cne->GetV().ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = cne->GetV().ValueOfVz(TIME) + wbx*lby-wby*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
							
				meff = a->Mass();
				g0 = DampingCoefficient(en, meff, k);
				
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
					xsi = a->GetElongationManager().GetElongation(cne->Numero(),Contact::Type::SphereCone,-9);
					computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
					a->GetElongationManager().AddElongation(xsi,cne->Numero(),Contact::Type::SphereCone,-9);
				}			
							
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
				
				// Ajout de la force et du moment
				a->AddForce(-Fx, -Fy, -Fz);
				a->AddMomemtum(-(lay*Fz-laz*Fy), -(laz*Fx-lax*Fz), -(lax*Fy-lay*Fx));

				cne->AddForce(Fx, Fy, Fz);
				cne->AddMomemtum((lby*Fz-lbz*Fy), (lbz*Fx-lbx*Fz), (lbx*Fy-lby*Fx));

				ctl->xi = a->X();
				ctl->yi = a->Y();
				ctl->zi = a->Z();
				ctl->xf = a->X() + lax;
				ctl->yf = a->Y() + lay;
				ctl->zf = a->Z() + laz;

				break;
			case Contact::Type::BodyCone: //13
				// Cas du contact entre une particule et un cone
				ba = ctl->ba;
				na = ctl->nba;
				cne = ctl->cn;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = ba->SphereRadius(na)*ctl->nx + (ba->SphereX(na) - ba->X());
				lay = ba->SphereRadius(na)*ctl->ny + (ba->SphereY(na) - ba->Y());
				laz = ba->SphereRadius(na)*ctl->nz + (ba->SphereZ(na) - ba->Z());
				ba->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
				
				lbx = ctl->px - cne->GetV().ox;
				lby = ctl->py - cne->GetV().oy;
				lbz = ctl->pz - cne->GetV().oz;
				wbx = cne->GetV().ValueOfWx(TIME);
				wby = cne->GetV().ValueOfWy(TIME);
				wbz = cne->GetV().ValueOfWz(TIME);
				Vbx = cne->GetV().ValueOfVx(TIME) + wby*lbz-wbz*lby;
				Vby = cne->GetV().ValueOfVy(TIME) + wbz*lbx-wbx*lbz;
				Vbz = cne->GetV().ValueOfVz(TIME) + wbx*lby-wby*lbx;
				
				// Calcul des vitesses local du contact
				computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);
				
				meff = ba->Mass();
				// Parametre du contact fonction de plan et de l'espece de grain en contact
				g0 = DampingCoefficient(en, meff, k);
				
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

					xsi = ba->GetElongationManager().GetElongation(cne->Numero(),Contact::Type::BodyCone,na,-9);
					computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
					ba->GetElongationManager().AddElongation(xsi,cne->Numero(),Contact::Type::BodyCone,na,-9);
				}
				
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
				
				// Ajout de la force et du moment

				ba->AddForce(-Fx, -Fy, -Fz);
				ba->AddMomemtum(-(lay*Fz-laz*Fy), -(laz*Fx-lax*Fz), -(lax*Fy-lay*Fx));

				cne->AddForce(Fx, Fy, Fz);
				cne->AddMomemtum((lby*Fz-lbz*Fy), (lbz*Fx-lbx*Fz), (lbx*Fy-lby*Fx));

				break;
			case Contact::Type::SphereElbow: //4
				// Cas du contact entre une sphere et un coude
				a = ctl->sa;
				elw = ctl->ew;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = -a->Radius()*ctl->nx;
				lay = -a->Radius()*ctl->ny;
				laz = -a->Radius()*ctl->nz;
				a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
				
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
				
				meff = a->Mass();
				g0 = DampingCoefficient(en, meff, k);
			
				// Dynamique
				if(ModelTg == 0){
					gt = 10000;
					// Calcul de N et T
					computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
				}
				// Statique
				else{
					xsi = a->GetElongationManager().GetElongation(elw->numero,Contact::Type::SphereElbow,-9);
					computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
					a->GetElongationManager().AddElongation(xsi,elw->numero,Contact::Type::SphereElbow,-9);
				}		
				
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

				// Ajout de la force et du moment
				a->AddForce(-Fx, -Fy, -Fz);
				a->AddMomemtum(-(lay*Fz-laz*Fy), -(laz*Fx-lax*Fz), -(lax*Fy-lay*Fx));


				break;
			case Contact::Type::BodyElbow: //14
				// Cas du contact entre une particule et un coude
				ba = ctl->ba;
				na = ctl->nba;
				elw = ctl->ew;

				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = -ba->SphereRadius(na)*ctl->nx + (ba->SphereX(na) - ba->X());
				lay = -ba->SphereRadius(na)*ctl->ny + (ba->SphereY(na) - ba->Y());
				laz = -ba->SphereRadius(na)*ctl->nz + (ba->SphereZ(na) - ba->Z());
				ba->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
				
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
							
				meff = ba->Mass();
				g0 = DampingCoefficient(en, meff, k);
							
				// Dynamique
				if(ModelTg == 0){
					gt = 10000;
					// Calcul de N et T
					computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
				}
				// Statique
				else{
					xsi = ba->GetElongationManager().GetElongation(elw->numero,Contact::Type::BodyElbow,na,-9);
					computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
					ba->GetElongationManager().AddElongation(xsi,elw->numero,Contact::Type::BodyElbow,na,-9);
				}					
				
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

				// Ajout de la force et du moment
				ba->AddForce(-Fx, -Fy, -Fz);
				ba->AddMomemtum(-(lay*Fz-laz*Fy), -(laz*Fx-lax*Fz), -(lax*Fy-lay*Fx));

				break;
            case Contact::Type::SphereHollowBall:
                // Cas du contact entre deux spheres
				a = ctl->sa;
				b = ctl->sb;
			    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lax = -a->Radius()*ctl->nx;
				lay = -a->Radius()*ctl->ny;
				laz = -a->Radius()*ctl->nz;
				a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
				
				lbx = -b->Radius()*ctl->nx;
				lby = -b->Radius()*ctl->ny;
				lbz = -b->Radius()*ctl->nz;

				b->PointVelocity(Vbx, Vby, Vbz, lbx, lby, lbz);
				
				// Calcul des vitesses local du contact
				computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);
                
				/* Cas du contact physique */
				if(ctl->delta <= 0){
					//Calcul de la masse effective
					meff = a->Mass()*b->Mass()/(a->Mass()+b->Mass());
					// Coefficient visqueux normale fonction de la masse effective meff, de la raideur k et de la restituion en
					g0 = DampingCoefficient(en, meff, k);
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
						xsi = a->GetElongationManager().GetElongation(b->Num(),Contact::Type::SphereHollowBall,-9);
						// Calcul de N, T et Xsi
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
						// Enregistrement du nouveau Xsi
						a->GetElongationManager().AddElongation(xsi,b->Num(),Contact::Type::SphereHollowBall,-9);
						b->GetElongationManager().AddElongation(xsi,a->Num(),Contact::Type::SphereHollowBall,-9);
					}
				}
				// Calcul de la force de contact
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
                
				// Incrementation de la force et du moment sur chaque corps
				a->AddForce(Fx, Fy, Fz);
				a->AddMomemtum((lay*Fz-laz*Fy), (laz*Fx-lax*Fz), (lax*Fy-lay*Fx));

				b->AddForce(-Fx, -Fy, -Fz);
				b->AddMomemtum(-(lby*Fz-lbz*Fy), -(lbz*Fx-lbx*Fz), -(lbx*Fy-lby*Fx));

				break;
            case Contact::Type::BodyHollowBall: //6
                // Cas du contact entre une sphere et une particules
				a = ctl->sa;
				bb = ctl->bb;
				nb = ctl->nbb;
				
				/* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
				lbx = -a->Radius()*ctl->nx;
				lby = -a->Radius()*ctl->ny;
				lbz = -a->Radius()*ctl->nz;
				a->PointVelocity(Vbx, Vby, Vbz, lbx, lby, lbz);
            
				lax = -bb->SphereRadius(nb)*ctl->nx + (bb->SphereX(nb) - bb->X());
				lay = -bb->SphereRadius(nb)*ctl->ny + (bb->SphereY(nb) - bb->Y());
				laz = -bb->SphereRadius(nb)*ctl->nz + (bb->SphereZ(nb) - bb->Z());
				bb->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
				
				// Calcul des vitesses local du contact
				computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);
                
				/* Cas du contact physique */
				if(ctl->delta <= 0){
					//Calcul de la masse effective
					meff = a->Mass()*bb->Mass()/(a->Mass()+bb->Mass());
					// Coefficient visqueux normale fonction de la masse effective meff, de la raideur k et de la restituion en
					g0 = DampingCoefficient(en, meff, k);
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
						xsi = bb->GetElongationManager().GetElongation(a->Num(),Contact::Type::BodyHollowBall,nb,-9);
						// Calcul de N, T et Xsi
						computeContactForceStatic(ctl, xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,dat.muD);
						// Enregistrement du nouveau Xsi
						a->GetElongationManager().AddElongation(xsi,bb->Num(),Contact::Type::BodyHollowBall,nb);
						bb->GetElongationManager().AddElongation(xsi,a->Num(),Contact::Type::BodyHollowBall,nb,-9);
					}
				}
				
				// Calcul de la force de contact
				ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
                
				// Incrementation de la force et du moment sur chaque corps
				a->AddForce(-Fx, -Fy, -Fz);
				a->AddMomemtum(-(lby*Fz-lbz*Fy), -(lbz*Fx-lbx*Fz), -(lbx*Fy-lby*Fx));

				bb->AddForce(Fx, Fy, Fz);
				bb->AddMomemtum((lay*Fz-laz*Fy), (laz*Fx-lax*Fz), (lax*Fy-lay*Fx));

                break;
		}
		ctl->Fx = Fx;
		ctl->Fy = Fy;
		ctl->Fz = Fz;
		ctl++;
	}
}
