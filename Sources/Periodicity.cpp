#include "../Includes/Periodicity.h"

#import "../Includes/Plan.h"
#import "../Includes/Sphere.h"
#import "../Includes/Body.h"

void PeriodicityPL(const int & Nsph, const int & Nbd, const int &Npl, std::vector<Sphere> & sph, std::vector<Body> & bd, std::vector<Plan> & pl) noexcept {
	int plp;
	double px,py,pz,pn,pt,ps;
	Sphere *b;
	
	for(int i = 0 ; i < Npl ; i++){
		if((plp = pl[i].periodic) != -9){
			for(int j = 0 ; j < pl[i].Nlist ; j++){
				b = &sph[pl[i].list[j]];
				if((b->Bodies()) == -9){
					px = b->X();
					py = b->Y();
					pz = b->Z();
					pn = (px-pl[i].X())*pl[i].Nx()+(py-pl[i].Y())*pl[i].Ny()+(pz-pl[i].Z())*pl[i].Nz();
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
	
	for(int i = 0 ; i < Npl ; i++){
		if((plp = pl[i].periodic) != -9){
			for(int j = 0 ; j < pl[i].Nlist ; j++){
				b = &sph[pl[i].list[j]];	
				if(b->Bodies() != -9){
					px = (b->GetBody())->X();
					py = (b->GetBody())->Y();
					pz = (b->GetBody())->Z();
					pn = (px-pl[i].X())*pl[i].Nx()+(py-pl[i].Y())*pl[i].Ny()+(pz-pl[i].Z())*pl[i].Nz();
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
