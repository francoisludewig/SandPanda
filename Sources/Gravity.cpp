#include "../Includes/Gravity.h"

Gravity::Gravity() noexcept :
G(9.81), ngx(0), ngy(0),	ngz(-1), ngx0(0), ngy0(0),
ngz0(-1), q0(1), q1(0), q2(0), q3(0) {}

void Gravity::LoadFromFile(FILE *ft) noexcept {
	fscanf(ft,"%lf\t%lf\t%lf",&ngx,&ngy,&ngz);
	fscanf(ft,"%lf\t%lf\t%lf\t%lf",&q0,&q1,&q2,&q3);
	wx.LoadFromFile(ft);
	wy.LoadFromFile(ft);
	wz.LoadFromFile(ft);
	fscanf(ft,"%lf",&G);
	
	printf("Gravity\n");
	printf("-------\n\n");
	printf("g = (%e,%e,%e)\n\n",ngx*G,ngy*G,ngz*G);
	
	ngx0 = ngx;
	ngy0 = ngy;
	ngz0 = ngz;
}

void Gravity::WriteToFile(FILE *ft) const noexcept {
	fprintf(ft,"%e\t%e\t%e\n",ngx,ngy,ngz);
	fprintf(ft,"%e\t%e\t%e\t%e\n",q0,q1,q2,q3);
	wx.WriteToFile(ft,0);
	wy.WriteToFile(ft,0);
	wz.WriteToFile(ft,0);
	fprintf(ft,"%e\n",G);
}

void Gravity::Move(double time,double dt) noexcept{
	double wlx,wly,wlz;
	double ql0,ql1,ql2,ql3;
	double p0,p1,p2,p3;
	// Rotation
	wlx = wx.Value(time)*dt;
	wly = wy.Value(time)*dt;
	wlz = wz.Value(time)*dt;
	
	double W = sqrt(wlx*wlx+wly*wly+wlz*wlz);
	double Ca = cos(W/2.),Sa = sin(W/2.);
	if(W != 0){
		wlx = wlx/W;
		wly = wly/W;
		wlz = wlz/W;
		
		p0 = Ca;
		p1 = dt*wlx/W*Sa;
		p2 = dt*wly/W*Sa;
		p3 = dt*wlz/W*Sa;
		
		ql0 = q0;
		ql1 = q1;
		ql2 = q2;
		ql3 = q3;
		
		if(p0 != 0 && ql0 != 0){
			q0 = ql0*p0 - ql1*p1 - ql2*p2 - ql3*p3;
			q1 = ql0*p1 + ql1*p0 - ql2*p3 + ql3*p2;
			q2 = ql0*p2 + ql1*p3 + ql2*p0 - ql3*p1;
			q3 = ql0*p3 - ql1*p2 + ql2*p1 + ql3*p0;
		}
		if(p0 != 0 && ql0 == 0){
			q0 = p0;
			q1 = p1;
			q2 = p2;
			q3 = p3;
		}
		
		// Ecriture de la nouvelle base locale via le quaternion
		double nx = 1 - 2*q2*q2 - 2*q3*q3;
		double ny = 2*q1*q2 + 2*q3*q0;
		double nz = 2*q1*q3 - 2*q2*q0;
		
		double tx = 2*q1*q2 - 2*q3*q0;
		double ty = 1 - 2*q1*q1 - 2*q3*q3;
		double tz = 2*q2*q3 + 2*q1*q0;
		
		double sx = 2*q1*q3 + 2*q2*q0;
		double sy = 2*q2*q3 - 2*q1*q0;
		double sz = 1 - 2*q1*q1 - 2*q2*q2;
		
		ngx = ngx0*nx + ngy0*tx + ngz0*sx;
		ngy = ngx0*ny + ngy0*ty + ngz0*sy;
		ngz = ngx0*nz + ngy0*tz + ngz0*sz;
	}
}
