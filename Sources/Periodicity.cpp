#include "../Includes/Periodicity.h"

#import "../Includes/Solids/Plan.h"
#import "../Includes/Solids/Sphere.h"
#import "../Includes/Solids/Body.h"

void PeriodicityPL(std::vector<Sphere> & sph, std::vector<Plan> & pl) noexcept {
	int plp;
	double px,py,pz,pn,pt,ps;
	Sphere *b;
	

	for(auto& plan : pl){
		if((plp = plan.Periodic()) != -9){
			for(int j = 0 ; j < plan.ListCount() ; j++){
				b = &sph[plan.List(j)];
				if((b->Bodies()) == -9){
					px = b->X();
					py = b->Y();
					pz = b->Z();
					pn = (px-plan.X())*plan.Nx()+(py-plan.Y())*plan.Ny()+(pz-plan.Z())*plan.Nz();
					pt = (px-pl[plp].X())*pl[plp].Tx()+(py-pl[plp].Y())*pl[plp].Ty()+(pz-pl[plp].Z())*pl[plp].Tz();
					ps = (px-pl[plp].X())*pl[plp].Sx()+(py-pl[plp].Y())*pl[plp].Sy()+(pz-pl[plp].Z())*pl[plp].Sz();
					if(pn < 0){
						b->X(pl[plp].X() + (-pn)*pl[plp].Nx() + pt*pl[plp].Tx() + ps*pl[plp].Sx());
						b->Y(pl[plp].Y() + (-pn)*pl[plp].Ny() + pt*pl[plp].Ty() + ps*pl[plp].Sy());
						b->Z(pl[plp].Z() + (-pn)*pl[plp].Nz() + pt*pl[plp].Tz() + ps*pl[plp].Sz());
					}
				}
			} 
		}
	}
	
	for(auto& plan : pl){
		if((plp = plan.Periodic()) != -9){
			for(int j = 0 ; j < plan.ListCount() ; j++){
				b = &sph[plan.List(j)];
				if(b->Bodies() != -9){
					px = (b->GetBody())->X();
					py = (b->GetBody())->Y();
					pz = (b->GetBody())->Z();
					pn = (px-plan.X())*plan.Nx()+(py-plan.Y())*plan.Ny()+(pz-plan.Z())*plan.Nz();
					pt = (px-pl[plp].X())*pl[plp].Tx()+(py-pl[plp].Y())*pl[plp].Ty()+(pz-pl[plp].Z())*pl[plp].Tz();
					ps = (px-pl[plp].X())*pl[plp].Sx()+(py-pl[plp].Y())*pl[plp].Sy()+(pz-pl[plp].Z())*pl[plp].Sz();
					if(pn < 0){
						(b->GetBody())->X(pl[plp].X() + (-pn)*pl[plp].Nx() + pt*pl[plp].Tx() + ps*pl[plp].Sx());
						(b->GetBody())->Y(pl[plp].Y() + (-pn)*pl[plp].Ny() + pt*pl[plp].Ty() + ps*pl[plp].Sy());
						(b->GetBody())->Z(pl[plp].Z() + (-pn)*pl[plp].Nz() + pt*pl[plp].Tz() + ps*pl[plp].Sz());
						(b->GetBody())->UpDateLinkedSphere(sph);
					}
				}
			}
		}
	}
}
