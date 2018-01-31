#include "../../Includes/Contact/ContactDetection.h"
//#include <omp.h>

#include <cmath>

#include "../../Includes/LinkedCells/SolidCells.h"
#include "../../Includes/LinkedCells/CellBounds.h"
#include "../../Includes/Solids/Velocity.h"
#include "../../Includes/Gravity.h"
#include "../../Includes/Solids/Plan.h"
#include "../../Includes/Solids/PlanR.h"
#include "../../Includes/Solids/Cone.h"
#include "../../Includes/Solids/Elbow.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/Body.h"
#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Data.h"
#include "../../Includes/Solids/HollowBall.h"

#include "../../Includes/Contact/ContactDetectorSphereVersusSphere.h"
#include "../../Includes/Contact/ContactDetectorSphereVersusPlan.h"
#include "../../Includes/Contact/ContactDetectorSphereVersusDisk.h"
#include "../../Includes/Contact/ContactDetectorSphereVersusCone.h"
#include "../../Includes/Contact/ContactDetectorSphereVersusElbow.h"
#include "../../Includes/Contact/ContactDetectorBodyVersusBody.h"


int ContactDetection::ContactSphBodyPeriodic(Sphere *a, Body *b, Contact *ct, int &Nct) noexcept {
	int Nctfound = 0;
	double Nx,Ny,Nz,xc,yc,zc,norme;
	// Vecteur normal joignant les centres de masse
	Nx = a->X2()-b->X();
	Ny = a->Y2()-b->Y();
	Nz = a->Z2()-b->Z();
	norme = sqrt(Nx*Nx+Ny*Ny+Nz*Nz);
	Nx = Nx/norme;
	Ny = Ny/norme;
	Nz = Nz/norme;

	// Point central du segment joignant les centres de masse
	xc = a->X2() - Nx*a->Radius();
	yc = a->Y2() - Ny*a->Radius();
	zc = a->Z2() - Nz*a->Radius();

	double delta[b->SphereCount()];
	// Liste des sph de b
	for(int i = b->SphereCount() ; i--;){
		delta[i] = (b->SphereX(i)-xc)*Nx+(b->SphereY(i)-yc)*Ny+(b->SphereZ(i)-zc)*Nz;
	}

	// Test Contact potentiel
	for(int i = b->SphereCount() ; i--;){
		if(delta[i] > -b->SphereRadius(i)){
			double px,py,pz,Q,P2;
			px = a->X2() - b->SphereX(i);
			py = a->Y2() - b->SphereY(i);
			pz = a->Z2() - b->SphereZ(i);
			Q = (a->Radius()+b->SphereRadius(i));
			P2 = px*px+py*py+pz*pz;
			if(P2 < Q*Q){
				P2 = sqrt(P2);
				ct[Nct].type = Contact::Type::SphereBody;// 10;
				ct[Nct].delta = P2-(a->Radius()+b->SphereRadius(i));

				ct[Nct].nx = px/P2;
				ct[Nct].ny = py/P2;
				ct[Nct].nz = pz/P2;

				ct[Nct].px = b->SphereX(i) + px/2.;
				ct[Nct].py = b->SphereY(i) + py/2.;
				ct[Nct].pz = b->SphereZ(i) + pz/2.;

				ct[Nct].sa = a;
				ct[Nct].bb = b;
				ct[Nct].nbb = i;
				Nct++;
				Nctfound++;
			}
		}
	}

	if(Nctfound == 0)
		return 0;
	else
		return 1;
}

void ContactDetection::ContactSphSphPeriodic(Sphere *a, Sphere *b, Contact *ct, int & Nct) noexcept {
	int rtn = 0;
	if(a->Num() != b->Num()){
		if((a->Bodies() == -9 && b->Bodies() == -9)){
			double px,py,pz,P2,Q;
			px = a->X2() - b->X();
			py = a->Y2() - b->Y();
			pz = a->Z2() - b->Z();
			P2 = px*px+py*py+pz*pz;
			Q = (a->Radius()+b->Radius());
			if(P2 < Q*Q){
				P2 = sqrt(P2);
				ct[Nct].type = Contact::Type::SphereSphere;// 0;
				ct[Nct].delta = P2-Q;
				ct[Nct].nx = px/P2;
				ct[Nct].ny = py/P2;
				ct[Nct].nz = pz/P2;
				ct[Nct].px = b->X() + px/2;
				ct[Nct].py = b->Y() + py/2;
				ct[Nct].pz = b->Z() + pz/2;
				ct[Nct].sa = a;
				ct[Nct].sb = b;
				Nct++;
			}
		}

		else{
			if(a->Bodies() != -9 && b->Bodies() == -9){
				rtn = ContactSphBodyPeriodic(b, a->GetBody(),ct,Nct);
			}
			if(a->Bodies() == -9 && b->Bodies() != -9){
				rtn = ContactSphBodyPeriodic(a, b->GetBody(),ct,Nct);
			}
			if(a->Bodies() != -9 && b->Bodies() != -9){
				double tpx,tpy,tpz;
				tpx = a->X();
				tpy = a->Y();
				tpz = a->Z();
				(a->GetBody())->X(a->X2());
				(a->GetBody())->Y(a->Y2());
				(a->GetBody())->Z(a->Z2());
				(a->GetBody())->UpDateLinkedSphereTp();

				rtn = ContactDetectorBodyVersusBody::Detect(a->GetBody(), b->GetBody(),ct,Nct,a->Radius(),b->Radius());
				(a->GetBody())->X(tpx);
				(a->GetBody())->Y(tpy);
				(a->GetBody())->Z(tpz);
				(a->GetBody())->UpDateLinkedSphereTp();

			}
		}
	}
}

void ContactDetection::ContactSphPlanPeriodic(std::vector<Sphere*>& llist, Plan & p, Plan & p2, Sphere *b, const double rmax) noexcept {
	double px,py,pz,pn,pt,ps;
	px = b->X();
	py = b->Y();
	pz = b->Z();
	pn = (px-p.X())*p.Nx()+(py-p.Y())*p.Ny()+(pz-p.Z())*p.Nz();
	pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
	ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
	if( (pn <= (b->Radius()+rmax)) && (pn > -2*b->Radius()) && (fabs(pt) < p.Dt()/2+b->Radius()) &&
			(fabs(ps) < p.Ds()/2+b->Radius()) ){
		pt = (px-p2.X())*p2.Tx()+(py-p2.Y())*p2.Ty()+(pz-p2.Z())*p2.Tz();
		ps = (px-p2.X())*p2.Sx()+(py-p2.Y())*p2.Sy()+(pz-p2.Z())*p2.Sz();
		b->X2(p2.X() + (-pn)*p2.Nx() + pt*p2.Tx() + ps*p2.Sx());
		b->Y2(p2.Y() + (-pn)*p2.Ny() + pt*p2.Ty() + ps*p2.Sy());
		b->Z2(p2.Z() + (-pn)*p2.Nz() + pt*p2.Tz() + ps*p2.Sz());
		llist[p.ListCount()] = b;
		p.List(p.ListCount(), b->Num());
		p.ListCount(p.ListCount() + 1);
	}
}

void ContactDetection::sphContactAll(std::vector<Sphere> & sph, Contact *ctl, int & Nctl) noexcept {
	int i,j;
	for(i = 0 ; i < sph.size() ; i++){
		for(j = i+1 ; j < sph.size() ; j++)
			ContactDetectorSphereVersusSphere::Detect(&sph[i],&sph[j],ctl,Nctl);
	}
}

void ContactDetection::sphContact(const CellBounds& cellBounds, Contact *ctl, int & Nctl, std::vector<Sphere*>& cell) noexcept {
	int l,m;
	Sphere *cand[5000],*anta;
	int Ncand;
	int num;

	int startx = cellBounds.StartX();
	int starty = cellBounds.StartY();
	int startz = cellBounds.StartZ();
	int endx = cellBounds.EndX();
	int endy = cellBounds.EndY();
	int endz = cellBounds.EndZ();
	int maxx = cellBounds.MaxX();
	int maxy = cellBounds.MaxY();
	int maxz = cellBounds.MaxZ();


	for(int i = startx ; i < endx ; i++){
		for(int j = starty ; j < endy ; j++){
			for(int k = startz ; k < endz ; k++){
				// num = i*Ny*Nz+j*Nz+k
				num = i*maxy*maxz+j*maxz+k;
				Ncand = 0;

				//printf("(%d,%d,%d) => %d\n",i,j,k,num);

				if((cand[Ncand] = cell[num]) != nullptr){
					//printf("(%d,%d,%d) => %d\n",i,j,k,num);
					do{
						//cand[Ncand]->affiche();
						Ncand++;
					}while((cand[Ncand] = cand[Ncand-1]->TDL()) != nullptr);

					// test dans le meme boite
					for(l = 0 ; l < Ncand ; l++){
						for(m = l+1 ; m < Ncand ; m++)
							ContactDetectorSphereVersusSphere::Detect(cand[l],cand[m],ctl,Nctl);
					}
					// Solo
					if(i+1 < maxx){
						//(i+1)*Ny*Nz+j*Nz+k = num+Ny*Nz
						if((anta = cell[num+maxy*maxz]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(j+1 < maxy){
						//i*Ny*Nz+(j+1)*Nz+k = num+Nz
						if((anta = cell[num+maxz]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(k+1 < maxz){
						//i*Ny*Nz+j*Nz+k+1 = num+1
						if((anta = cell[num+1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					// plan XY
					if(i+1 < maxx && j+1 < maxy){
						//(i+1)*Ny*Nz+(j+1)*Nz+k = num + Ny*Nz + Nz = num + Nz*(Ny+1)
						if((anta = cell[num+maxz*(maxy+1)]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(i+1 < maxx && j-1 >= 0){
						//(i+1)*Ny*Nz+(j-1)*Nz+k = num + Ny*Nz - Nz = num + Nz*(Ny-1)
						if((anta = cell[num+maxz*(maxy-1)]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					// plan XZ
					if(i+1 < maxx && k+1 < maxz){
						//(i+1)*Ny*Nz+j*Nz+k+1 = num + Ny*Nz + 1
						if((anta = cell[num+maxy*maxz+1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(i+1 < maxx && k-1 >= 0){
						//(i+1)*Ny*Nz+j*Nz+k-1 = num + Ny*Nz - 1
						if((anta = cell[num+maxy*maxz-1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					// plan YZ
					if(j+1 < maxy && k+1 < maxz){
						//i*Ny*Nz+(j+1)*Nz+k+1 = num + Nz + 1
						if((anta = cell[num+maxz+1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(j+1 < maxy && k-1 >= 0){
						//i*Ny*Nz+(j+1)*Nz+k-1 = num + Nz - 1
						if((anta = cell[num+maxz-1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					// tripple
					if(i+1 < maxx && j+1 < maxy && k+1 < maxz){
						//(i+1)*Ny*Nz+(j+1)*Nz+k+1 = num + Nz*(Ny + 1) + 1
						if((anta = cell[num+maxz*(maxy+1)+1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(i+1 < maxx && j+1 < maxy && k-1 >= 0){
						//(i+1)*Ny*Nz+(j+1)*Nz+k-1 = num + Nz*(Ny + 1) - 1
						if((anta = cell[num+maxz*(maxy+1)-1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(i+1 < maxx && j-1 >= 0 && k+1 < maxz){
						//(i+1)*Ny*Nz+(j-1)*Nz+k+1 = num + Nz*(Ny - 1) + 1
						if((anta = cell[num+maxz*(maxy-1)+1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(i-1 >= 0 && j+1 < maxy && k+1 < maxz){
						//(i-1)*Ny*Nz+(j+1)*Nz+k+1 = num + Nz*(1-Ny)+1
						if((anta = cell[num+maxz*(1-maxy)+1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++)
									ContactDetectorSphereVersusSphere::Detect(cand[l],anta,ctl,Nctl);
							}while((anta = anta->TDL()) != nullptr);
						}
					}
				}
			}
		}
	}
}

void ContactDetection::sphPlanContact(int & Nct, std::vector<Sphere> & sph, std::vector<Plan> & pl, Contact *ct, std::vector<Sphere*>& cell, const SolidCells& solidCells, const double rmax) noexcept {
	Sphere *anta;
	std::vector<int> control(pl.size(), 0);

	for(int i = 0 ; i < pl.size() ; i++){
		if(pl[i].Periodic() == -9)
			control[i] = 1;
		else
			control[i] = 0;
	}

	for(int i = 0 ; i < pl.size() ; i++){
		const auto& cellIndex = solidCells.PlanCells(i);
		if(pl[i].Periodic() == -9){
			for(int j = 0 ; j < cellIndex.Size() ; ++j){
				if((anta = cell[cellIndex[j]]) != nullptr){
					do{
						ContactDetectorSphereVersusPlan::Detect(pl[i], anta, ct, Nct);
					}while((anta = anta->TDL()) != nullptr);
				}
			}
		}
		else{
			if(control[i] == 0){
				//Sphere *llistI[Nsph];
				std::vector<Sphere*> llistI(sph.size(), nullptr);

				pl[i].ListCount(0);
				for(int j = 0 ; j < cellIndex.Size() ; ++j){
					if((anta = cell[cellIndex[j]]) != nullptr){
						do{
							ContactSphPlanPeriodic(llistI, pl[i], pl[pl[i].Periodic()], anta, rmax);
						}while((anta = anta->TDL()) != nullptr);
					}
				}

				std::vector<Sphere*> llistK(sph.size(), nullptr);
				int k = pl[i].Periodic();
				const auto& cellIndexPeriodic = solidCells.PlanCells(k);
				pl[k].ListCount(0);
				for(int j = 0 ; j < cellIndexPeriodic.Size() ; j++){
					if((anta = cell[cellIndexPeriodic[j]]) != nullptr){
						do{
							ContactSphPlanPeriodic(llistK, pl[k], pl[pl[k].Periodic()], anta, rmax);
						}while((anta = anta->TDL()) != nullptr);
					}
				}

				for(int j = 0 ; j < pl[i].ListCount() ; j++){
					for(int l = 0 ; l < pl[k].ListCount() ; l++){
						ContactSphSphPeriodic(llistI[j], llistK[l], ct, Nct);
					}
				}
				control[i] = 1;
				control[k] = 1;
			}
		}
	}
}

void ContactDetection::sphPlanRContact(int & Nct, std::vector<PlanR> & plr, Contact *ct, std::vector<Sphere*>& cell, const SolidCells& solidCells) noexcept {
	Sphere *anta;
	for(auto& disk: plr) {
		const auto& cellIndex = solidCells.PlanRCells(disk.Numero());
		for(int j = 0 ; j < cellIndex.Size() ; j++){
			if((anta = cell[cellIndex[j]]) != nullptr){
				do{
					ContactDetectorSphereVersusDisk::Detect(disk, anta, ct, Nct);
				}while((anta = anta->TDL()) != nullptr);
			}
		}
	}
}

void ContactDetection::sphConeContact(int & Nct, std::vector<Cone> & co, Contact *ct, std::vector<Sphere*>& cell, const SolidCells& solidCells)  noexcept {
	Sphere *anta;
	for(auto& cone : co) {
		const auto& cellIndex = solidCells.ConeCells(cone.Numero());
		for(int j = 0 ; j < cellIndex.Size() ; j++){
			if((anta = cell[cellIndex[j]]) != nullptr){
				do{
					ContactDetectorSphereVersusCone::Detect(cone, anta, ct, Nct);
				}while((anta = anta->TDL()) != nullptr);
			}
		}
	}
}

void ContactDetection::sphElbowContact(int & Nct, std::vector<Elbow> & elb, Contact *ct, std::vector<Sphere*>& cell, const SolidCells& solidCells)  noexcept {
	Sphere *anta;
	for(auto& elbow : elb) {
		const auto& cellIndex = solidCells.ElbowCells(elbow.numero);
		for(int j = 0 ; j < cellIndex.Size() ; j++){
			if((anta = cell[cellIndex[j]]) != nullptr){
				do{
					ContactDetectorSphereVersusElbow::Detect(elbow, anta, ct, Nct);
				}while((anta = anta->TDL()) != nullptr);
			}
		}
	}
}

void ContactDetection::sphHollowBallContact(int & Nct, std::vector<HollowBall> & hb, Contact *ct)  noexcept {

	for(auto& hollowBall : hb) {
		hollowBall.ContactDetectionInHollowBall(ct,Nct);
		hollowBall.ContactDetectionWithHollowBall(ct,Nct);
	}
}

void ContactDetection::sphContainer(std::vector<Sphere> & sph, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, std::vector<HollowBall> & hb, int & Nct, Contact *ct, std::vector<Sphere*>& cell, const SolidCells& solidCells, const double rmax) noexcept {
	sphPlanContact(Nct, sph, pl, ct, cell, solidCells,rmax);
	sphPlanRContact(Nct, plr, ct, cell, solidCells);
	sphConeContact(Nct, co, ct, cell, solidCells);
	sphElbowContact(Nct, elb, ct, cell, solidCells);
	sphHollowBallContact(Nct, hb, ct);
}

