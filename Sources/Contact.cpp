#include "../Includes/Contact.h"

#include "../Includes/Sphere.h"
#include "../Includes/Plan.h"
#include "../Includes/PlanR.h"
#include "../Includes/Cone.h"
#include "../Includes/Elbow.h"
#include "../Includes/Elongation.h"
#include "../Includes/Body.h"

#include <cmath>

Contact::Contact() noexcept :
type(-9), px(0), py(0),	pz(0), delta(0),	sa(nullptr),	sb(nullptr),	ba(nullptr),bb(nullptr), nba(-9), nbb(-9), pa(nullptr),
par(nullptr), cn(nullptr), ew(nullptr) {}

Contact::~Contact() noexcept {}

void Contact::TimeStepInitialization() noexcept {
	type = -9;
	px = 0;
	py = 0;
	pz = 0;
	delta = 0;
	sa = nullptr;
	sb = nullptr;
	pa = nullptr;
	par = nullptr;
	cn = nullptr;
	ew = nullptr;
	xsil.Reset();
}

void Contact::inFile(FILE *ft) noexcept {
    fprintf(ft,"Contact : type = %d\n",type);
	switch (type) {
		case 0:
			if(sa) fprintf(ft,"Sph : %d\n",sa->Num());
			if(sb) fprintf(ft,"Sph : %d\n",sb->Num());
			break;
		case 1:
			if(sa) fprintf(ft,"Sph : %d\n",sa->Num());
			if(pa) fprintf(ft,"Plan : %d\n",pa->Numero());
			break;
		default:
			break;
	}
    fprintf(ft,"\n");
}

void Contact::Display() const noexcept {
	printf("Contact : type = %d\n",type);
	switch (type) {
		case 0:			
			if(sa) printf("Sph : %d\n",sa->Num());
			if(sb) printf("Sph : %d\n",sb->Num());
			break;
		case 1:	
			if(sa) printf("Sph : %d\n",sa->Num());
			if(pa) printf("Plan : %d\n",pa->Numero());
			break;
		default:
			break;
	}
  printf("\n");
	printf("F = (%e,%e,%e)\n",Fx,Fy,Fz);
}

void Contact::WriteOutFile(FILE *ft,double time) const noexcept {
	fprintf(ft,"%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",type,delta,xi,yi,zi,xf,yf,zf);
}

void Contact::WriteOutFileDetails(FILE *ft,double time) const noexcept {
	 switch(type){
		case 0:
   fprintf(ft,"%d\t%d\t%d\t%e\t%e\t%e\t%e\t%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",type,sa->Num(),sb->Num(),px,py,pz,delta,Stick,nx,ny,nz,tx,ty,tz,Fx,Fy,Fz);
	 break;
		case 10:
	 fprintf(ft,"%d\t%d\t%d\t%e\t%e\t%e\t%e\t%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",type,sa->Num(),bb->Num(),px,py,pz,delta,Stick,nx,ny,nz,tx,ty,tz,Fx,Fy,Fz);
	 break;
		case 20:
	 fprintf(ft,"%d\t%d\t%d\t%e\t%e\t%e\t%e\t%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",type,ba->Num(),bb->Num(),px,py,pz,delta,Stick,nx,ny,nz,tx,ty,tz,Fx,Fy,Fz);
	 break;
		case 1:
	 fprintf(ft,"%d\t%d\t%d\t%e\t%e\t%e\t%e\t%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",type,sa->Num(),pa->Numero(),px,py,pz,delta,Stick,nx,ny,nz,tx,ty,tz,Fx,Fy,Fz);
	 break;
		case 11:
	 fprintf(ft,"%d\t%d\t%d\t%e\t%e\t%e\t%e\t%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",type,ba->Num(),pa->Numero(),px,py,pz,delta,Stick,nx,ny,nz,tx,ty,tz,Fx,Fy,Fz);
	 break;
		case 2:
	 fprintf(ft,"%d\t%d\t%d\t%e\t%e\t%e\t%e\t%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",type,sa->Num(),par->Numero(),px,py,pz,delta,Stick,nx,ny,nz,tx,ty,tz,Fx,Fy,Fz);
	 break;
		case 12:
	 fprintf(ft,"%d\t%d\t%d\t%e\t%e\t%e\t%e\t%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",type,ba->Num(),par->Numero(),px,py,pz,delta,Stick,nx,ny,nz,tx,ty,tz,Fx,Fy,Fz);
	 break;
		case 3:
	 fprintf(ft,"%d\t%d\t%d\t%e\t%e\t%e\t%e\t%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",type,sa->Num(),cn->Numero(),px,py,pz,delta,Stick,nx,ny,nz,tx,ty,tz,Fx,Fy,Fz);
	 break;
		case 13:
	 fprintf(ft,"%d\t%d\t%d\t%e\t%e\t%e\t%e\t%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",type,ba->Num(),cn->Numero(),px,py,pz,delta,Stick,nx,ny,nz,tx,ty,tz,Fx,Fy,Fz);
	 break;
		case 4:
	 fprintf(ft,"%d\t%d\t%d\t%e\t%e\t%e\t%e\t%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",type,sa->Num(),ew->numero,px,py,pz,delta,Stick,nx,ny,nz,tx,ty,tz,Fx,Fy,Fz);
	 break;
		case 14:
	 fprintf(ft,"%d\t%d\t%d\t%e\t%e\t%e\t%e\t%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",type,ba->Num(),ew->numero,px,py,pz,delta,Stick,nx,ny,nz,tx,ty,tz,Fx,Fy,Fz);
	 break;
	 };
}


