/*
 *  Plan.cpp
 *
 *
 *  Created by fanfan on 14/07/10.
 *  Copyright 2010 ULg. All rights reserved.
 *
 */

#include "../../Includes/Container.h"
#include "../../Includes/Object/Sphere.h"

Container::Container(){
	x = 0;
	y = 0;
	z = 0;
	nx = 1;
	ny = 0;
	nz = 0;
	tx = 0;
	ty = 1;
	tz = 0;
	sx = 0;
	sy = 0;
	sz = 1;
	vx = 0;
	vy = 0;
	vz = 0;
	wx = 0;
	wy = 0;
	wz = 0;
	Fx = 0;
	Fy = 0;
	Fz = 0;
	Mx = 0;
	My = 0;
	Mz = 0;
	Fcx = 0;
	Fcy = 0;
	Fcz = 0;
	Mcx = 0;
	Mcy = 0;
	Mcz = 0;
	xMemory = 0;
	yMemory = 0;
	zMemory = 0;
	Mass = 1;
	In = 1;
	It = 1;
	Is = 1;
	activeGravity = 0;
	ControlGB = 1;
	//NCell = 0;
	Ngb = 0;
	Force = 0;
	NCell = 0;
	Cell = NULL;
	NCell2 = 0;
	Cell2 = NULL;
    vcell = NULL;
}

Container::~Container(){
	if(Ngb != 0){
		delete [] num;
		delete [] GBx;
		delete [] GBy;
		delete [] GBz;
	}
    /*
     if(NCell != 0){
     for(int i = 0 ; i < NCell ; i++)
     delete vcell[i];
     
     delete []vcell;
     }
     */
}

Cell** Container::getVcell(){
    return vcell;
}

void Container::readFromFile(FILE *ft){
	fscanf(ft,"%lf\t%lf\t%lf",&x,&y,&z);
	fscanf(ft,"%lf\t%lf\t%lf\t%lf",&q0,&q1,&q2,&q3);
    /*
     fscanf(ft,"%lf\t%lf\t%lf\n",&nx,&ny,&nz);
     fscanf(ft,"%lf\t%lf\t%lf\n",&tx,&ty,&tz);
     fscanf(ft,"%lf\t%lf\t%lf\n",&sx,&sy,&sz);
     */
    
    
    ComputeBase();
    /*
    nx = 1 - 2*q2*q2 - 2*q3*q3;
	ny = 2*q1*q2 + 2*q3*q0;
	nz = 2*q1*q3 - 2*q2*q0;
	
	tx = 2*q1*q2 - 2*q3*q0;
	ty = 1 - 2*q1*q1 - 2*q3*q3;
	tz = 2*q2*q3 + 2*q1*q0;
	
	sx = 2*q1*q3 + 2*q2*q0;
	sy = 2*q2*q3 - 2*q1*q0;
	sz = 1 - 2*q1*q1 - 2*q2*q2;
 */
    
	V.ReadFromFile(ft);
}

void Container::ComputeBase(){
    nx = 1 - 2*q2*q2 - 2*q3*q3;
	ny = 2*q1*q2 + 2*q3*q0;
	nz = 2*q1*q3 - 2*q2*q0;
	
	tx = 2*q1*q2 - 2*q3*q0;
	ty = 1 - 2*q1*q1 - 2*q3*q3;
	tz = 2*q2*q3 + 2*q1*q0;
	
	sx = 2*q1*q3 + 2*q2*q0;
	sy = 2*q2*q3 - 2*q1*q0;
	sz = 1 - 2*q1*q1 - 2*q2*q2;
    
    // Elimination des erreurs
    double max = fabs(nx);
    if(max < fabs(ny))max = fabs(ny);
    if(max < fabs(nz))max = fabs(nz);
    
    if(fabs(nx)/max < pow(10.,-15))nx = 0;
    if(fabs(ny)/max < pow(10.,-15))ny = 0;
    if(fabs(nz)/max < pow(10.,-15))nz = 0;
    
    max = fabs(tx);
    if(max < fabs(ty))max = fabs(ty);
    if(max < fabs(tz))max = fabs(tz);
    
    if(fabs(tx)/max < pow(10.,-15))tx = 0;
    if(fabs(ty)/max < pow(10.,-15))ty = 0;
    if(fabs(tz)/max < pow(10.,-15))tz = 0;

    max = fabs(sx);
    if(max < fabs(sy))max = fabs(sy);
    if(max < fabs(sz))max = fabs(sz);
    
    if(fabs(sx)/max < pow(10.,-15))sx = 0;
    if(fabs(sy)/max < pow(10.,-15))sy = 0;
    if(fabs(sz)/max < pow(10.,-15))sz = 0;
}


void Container::readAccelerationFromFile(FILE *ft){
	fscanf(ft,"%lf\t%d",&Mass,&activeGravity);
	fscanf(ft,"%lf\t%lf\t%lf",&Fcx,&Fcy,&Fcz);
	fscanf(ft,"%lf\t%lf\t%lf",&Mcx,&Mcy,&Mcz);
	if(Fcx == 0 && Fcy == 0 && Fcz == 0 && Mcx == 0 && Mcy == 0 && Mcz == 0 && activeGravity == 0)
		Force = 0;
	else
		Force = 1;
}

void Container::writeAccelerationFromFile(FILE *ft){
	fprintf(ft,"%e\t%d\n",Mass,activeGravity);
	fprintf(ft,"%e\t%e\t%e\n",Fcx,Fcy,Fcz);
	fprintf(ft,"%e\t%e\t%e\n",Mcx,Mcy,Mcz);
}

void Container::writeToFile(FILE *ft){
	fprintf(ft,"%e\t%e\t%e\n",x,y,z);
    fprintf(ft,"%e\t%e\t%e\t%e\n",q0,q1,q2,q3);
    /*
     fprintf(ft,"%e\t%e\t%e\n",nx,ny,nz);
     fprintf(ft,"%e\t%e\t%e\n",tx,ty,tz);
     fprintf(ft,"%e\t%e\t%e\n",sx,sy,sz);
     */
	V.WriteToFile(ft,0);
}

void Container::writeOutFile(FILE *ft, int mode){
	if(mode == 0){
		fprintf(ft,"%e\t%e\t%e\n",x,y,z);
        fprintf(ft,"%e\t%e\t%e\t%e\n",q0,q1,q2,q3);
        /*
         fprintf(ft,"%e\t%e\t%e\n",nx,ny,nz);
         fprintf(ft,"%e\t%e\t%e\n",tx,ty,tz);
         fprintf(ft,"%e\t%e\t%e\n",sx,sy,sz);
         */
	}
	else{
		fwrite(&x, sizeof(double), 1, ft);
		fwrite(&y, sizeof(double), 1, ft);
		fwrite(&z, sizeof(double), 1, ft);
		
		fwrite(&nx, sizeof(double), 1, ft);
		fwrite(&ny, sizeof(double), 1, ft);
		fwrite(&nz, sizeof(double), 1, ft);
		
		fwrite(&tx, sizeof(double), 1, ft);
		fwrite(&ty, sizeof(double), 1, ft);
		fwrite(&tz, sizeof(double), 1, ft);
		
		fwrite(&sx, sizeof(double), 1, ft);
		fwrite(&sy, sizeof(double), 1, ft);
		fwrite(&sz, sizeof(double), 1, ft);
	}
	V.WriteToFile(ft,mode);
}

void Container::affiche(){
	printf("%e\t%e\t%e\n",x,y,z);
	printf("%e\t%e\t%e\n",nx,ny,nz);
	printf("%e\t%e\t%e\n",tx,ty,tz);
	printf("%e\t%e\t%e\n",sx,sy,sz);
	V.Display();
}

void Container::UpdateGravityForceFromGB(int & Nsph,vector<Sphere> & sph, Gravity gt){
	if(activeGravity == 1){
		if(Ngb != 0){
			Fx = 0;
			Fy = 0;
			Fz = 0;
			for(int i = 0 ; i < Ngb ; i++){
				if(gt.ngx*gt.G != 0)
					Fx += sph[num[i]].Force().ComponantX();
				if(gt.ngy*gt.G != 0)
					Fy += sph[num[i]].Force().ComponantY();
				if(gt.ngz*gt.G != 0)
					Fz += sph[num[i]].Force().ComponantZ();
			}
		}
	}
}

void Container::upDateGravityVelocity(double time, double dt, Gravity gt){
	if(activeGravity == 1){
		if(gt.ngx*gt.G != 0)
			vx += ((Fx)/Mass + gt.ngx*gt.G)*dt;
		if(gt.ngy*gt.G != 0)
			vy += ((Fy)/Mass + gt.ngy*gt.G)*dt;
		if(gt.ngz*gt.G != 0)
			vz += ((Fz)/Mass + gt.ngz*gt.G)*dt;
	}
}


void Container::upDateVelocity(double time, double dt, Gravity gt){
	double Mlx,Mly,Mlz,Mn,Mt,Ms,wn,wt,ws;
	
	if(activeGravity == 1){
		if(Fcx != 0 || gt.ngx*gt.G != 0)
			vx += ((Fcx+Fx)/Mass + gt.ngx*gt.G)*dt;
		else
			vx = V.VxValue(time);
		if(Fcy != 0 || gt.ngy*gt.G != 0)
			vy += ((Fcy+Fy)/Mass + gt.ngy*gt.G)*dt;
		else
			vy = V.VyValue(time);
		if(Fcz != 0 || gt.ngz*gt.G != 0)
			vz += ((Fcz+Fz)/Mass + gt.ngz*gt.G)*dt;
		else
			vz = V.VzValue(time);
	}
	else{
		if(Fcx != 0)
			vx += (Fcx+Fx)/Mass*dt;
		else
			vx = V.VxValue(time);
		if(Fcy != 0)
			vy += (Fcy+Fy)/Mass*dt;
		else
			vy = V.VyValue(time);
		if(Fcz != 0)
			vz += (Fcz+Fz)/Mass*dt;
		else
			vz = V.VzValue(time);
	}
	
	// Composantes de la force = somme contrainte + reaction
	Mlx = Mcx+Mx;
	Mly = Mcy+My;
	Mlz = Mcz+Mz;
	// Projection local
	Mn = Mlx*nx+Mly*ny+Mlz*nz;
	Mt = Mlx*tx+Mly*ty+Mlz*tz;
	Ms = Mlx*sx+Mly*sy+Mlz*sz;
	// Vitesse angulaire locale
	wn = Mn/In*dt;
	wt = Mt/It*dt;
	ws = Ms/Is*dt;
	// Incrementation dans la base globale
	if(Mcx != 0)
		wx += (wn*nx+wt*tx+ws*sx);
	else
		wx = V.WxValue(time);
	if(Mcy != 0)
		wy += (wn*ny+wt*ty+ws*sy);
	else
		wy = V.WyValue(time);
	
	if(Mcz != 0)
		wz += (wn*nz+wt*tz+ws*sz);
	else
		wz = V.WzValue(time);
}


void Container::UpdateForceFromGB(int & Nsph,vector<Sphere> & sph){
	if(Ngb != 0){
		Fx = 0;
		Fy = 0;
		Fz = 0;
		for(int i = 0 ; i < Ngb ; i++){
			Fx += sph[num[i]].Force().ComponantX();
			Fy += sph[num[i]].Force().ComponantY();
			Fz += sph[num[i]].Force().ComponantZ();
		}
	}
}

void Container::moveGravity(double dt, Gravity gt){
	if(activeGravity == 1){
		// Translation
		if(gt.ngx*gt.G != 0)
			x += vx*dt;
		if(gt.ngy*gt.G != 0)
			y += vy*dt;
		if(gt.ngz*gt.G != 0)
			z += vz*dt;
	}
}

void Container::move(double dt){
    double lx,ly,lz;
    //double wlx,wly,wlz;
    double a,sa,ca,ql0,ql1,ql2,ql3,p0,p1,p2,p3;
    // Translation
    x += vx*dt;
    y += vy*dt;
    z += vz*dt;
    // Rotation
    /*
     wlx = wx*dt;
     wly = wy*dt;
     wlz = wz*dt;
     */
    a = sqrt(wx*wx+wy*wy+wz*wz)*dt;
    if(a != 0){
        sa = sin(a/2);
        ca = cos(a/2);
        p0 = ca;
        p1 = dt*wx/a*sa;
        p2 = dt*wy/a*sa;
        p3 = dt*wz/a*sa;
        
        ql0 = q0;
        ql1 = q1;
        ql2 = q2;
        ql3 = q3;
        
        q0 = ql0*p0 - ql1*p1 - ql2*p2 - ql3*p3;
        q1 = ql0*p1 + ql1*p0 - ql2*p3 + ql3*p2;
        q2 = ql0*p2 + ql1*p3 + ql2*p0 - ql3*p1;
        q3 = ql0*p3 - ql1*p2 + ql2*p1 + ql3*p0;
        
        nx = 1 - 2*q2*q2 - 2*q3*q3;
        ny = 2*q1*q2 + 2*q3*q0;
        nz = 2*q1*q3 - 2*q2*q0;
        
        tx = 2*q1*q2 - 2*q3*q0;
        ty = 1 - 2*q1*q1 - 2*q3*q3;
        tz = 2*q2*q3 + 2*q1*q0;
        
        sx = 2*q1*q3 + 2*q2*q0;
        sy = 2*q2*q3 - 2*q1*q0;
        sz = 1 - 2*q1*q1 - 2*q2*q2;
        
        
        lx = (x-V.RotationOriginX());
        ly = (y-V.RotationOriginY());
        lz = (z-V.RotationOriginZ());
        
        x = ((1 - 2*p2*p2 - 2*p3*p3)*lx + (2*p1*p2 - 2*p3*p0)*ly     + (2*p1*p3 + 2*p2*p0)*lz)     + V.RotationOriginX();
        y = ((2*p1*p2 + 2*p3*p0)*lx     + (1 - 2*p1*p1 - 2*p3*p3)*ly + (2*p2*p3 - 2*p1*p0)*lz)     + V.RotationOriginY();
        z = ((2*p1*p3 - 2*p2*p0)*lx     + (2*p2*p3 + 2*p1*p0)*ly     + (1 - 2*p1*p1 - 2*p2*p2)*lz) + V.RotationOriginZ();
    }
}

void Container::upDateGravityLinkedSphere(vector<Sphere> & sph, double time, Gravity gt){
	if(activeGravity == 1){
		double newX, newY, newZ;
		for(int i = 0 ; i < Ngb ; i++){
			if(gt.ngx*gt.G != 0)
				newX = x;
			else
				newX = sph[num[i]].Position().CoordinateX();
			if(gt.ngy*gt.G != 0)
				newY = y;
			else
				newY = sph[num[i]].Position().CoordinateY();
			if(gt.ngz*gt.G != 0)
				newZ = z;
			else
				newZ = sph[num[i]].Position().CoordinateZ();
			
			sph[num[i]].SetPosition(newX, newY, newZ);
		}
	}
}

void Container::upDateLinkedSphere(vector<Sphere> & sph, double time, Gravity gt){
	double opx,opy,opz;
	// Prise de la vitesse imposee uniquement si aucune force est imposee
	if(Fcx == 0 && gt.ngx*gt.G == 0)
		vx = V.VxValue(time);
	if(Fcy == 0 && gt.ngy*gt.G == 0)
		vy = V.VyValue(time);
	if(Fcz == 0 && gt.ngz*gt.G == 0)
		vz = V.VzValue(time);
	if(Mcx == 0)
		wx = V.WxValue(time);
	if(Mcy == 0)
		wy = V.WyValue(time);
	if(Mcz == 0)
		wz = V.WzValue(time);
	
	for(int i = 0 ; i < Ngb ; i++){
		opx = GBx[i] - x;
		opy = GBy[i] - y;
		opz = GBz[i] - z;
		
		sph[num[i]].SetTranslationalVelocity(vx + (wy*opz-wz*opy),
																				 vy + (wz*opx-wx*opz),
																				 vz + (wx*opy-wy*opx));
		sph[num[i]].SetRotationalVelocity(wx, wy, wz);
		// Mise a jour des gB uniquement si autorise
		if(ControlGB == 1){
			sph[num[i]].SetPosition(x + GBx[i]*nx + GBy[i]*tx + GBz[i]*sx,
															y + GBx[i]*ny + GBy[i]*ty + GBz[i]*sy,
															z + GBx[i]*nz + GBy[i]*tz + GBz[i]*sz);
		}
	}
}

void Container::setControlGB(int v, vector<Sphere> & sph){
	if(v == 0 || v == 1){
		ControlGB = v;
		if(ControlGB == 1){
			for(int i = 0 ; i < Ngb ; i++)
				sph[num[i]].autoIntegrate = 0;
		}
		if(ControlGB == 0){
			for(int i = 0 ; i < Ngb ; i++)
				sph[num[i]].autoIntegrate = 1;
		}
	}
}


void Container::upDateGravityVelocityLinkedSphere(vector<Sphere> & sph, double time, Gravity gt){
	double opx,opy,opz;
	if(activeGravity == 1){
		if(gt.ngx*gt.G == 0)
			vx = V.VxValue(time);
		if(gt.ngy*gt.G == 0)
			vy = V.VyValue(time);
		if(gt.ngz*gt.G == 0)
			vz = V.VzValue(time);
	}
	for(int i = 0 ; i < Ngb ; i++){
		opx = GBx[i] - x;
		opy = GBy[i] - y;
		opz = GBz[i] - z;
		
		
		sph[num[i]].SetTranslationalVelocity(vx + (wy*opz-wz*opy),
																				 vy + (wz*opx-wx*opz),
																				 vz + (wx*opy-wy*opx));
		sph[num[i]].SetRotationalVelocity(wx, wy, wz);
	}
}


void Container::upDateVelocityLinkedSphere(vector<Sphere> & sph, double time){
	double opx,opy,opz;
	if(Force == 0 && activeGravity == 0){
		vx = V.VxValue(time);
		vy = V.VyValue(time);
		vz = V.VzValue(time);
		wx = V.WxValue(time);
		wy = V.WyValue(time);
		wz = V.WzValue(time);
	}
	for(int i = 0 ; i < Ngb ; i++){
		opx = GBx[i] - x;
		opy = GBy[i] - y;
		opz = GBz[i] - z;
		
		sph[num[i]].SetTranslationalVelocity(vx + (wy*opz-wz*opy),
																				 vy + (wz*opx-wx*opz),
																				 vz + (wx*opy-wy*opx));
		
		sph[num[i]].SetRotationalVelocity(wx, wy, wz);
	}
}


void Container::OnOffGravity(bool OnOff){
	if(OnOff){
		activeGravity = 1;
	}
	else{
		activeGravity = 0;
		vx = 0;
		vy = 0;
		vz = 0;
	}
}

void Container::SetVelocityToZero(){
	vx = 0;
	vy = 0;
	vz = 0;
	wx = 0;
	wy = 0;
	wz = 0;
}

double Container::Vmax(){
    return(V.TranslationalMaximum());   
}

double Container::Wmax(){
	return(V.RotationalMaximum());
}

double Container::Delay(){
	return(V.Period());
}

void Container::setVx(double newA0, double newA1, double newW, double newPhi){
	V.SetVxParameters(newA0, newA1, newW, newPhi);
}

void Container::setWx(double newA0, double newA1, double newW, double newPhi){
	V.SetWxParameters(newA0, newA1, newW, newPhi);
}

void Container::setVy(double newA0, double newA1, double newW, double newPhi){
	V.SetVyParameters(newA0, newA1, newW, newPhi);
}

void Container::setWy(double newA0, double newA1, double newW, double newPhi){
	V.SetWyParameters(newA0, newA1, newW, newPhi);
	
}

void Container::setVz(double newA0, double newA1, double newW, double newPhi){
	V.SetVzParameters(newA0, newA1, newW, newPhi);
}

void Container::setWz(double newA0, double newA1, double newW, double newPhi){
	V.SetWzParameters(newA0, newA1, newW, newPhi);
}

double Container::ValueOfVx(double t){
	return(V.VxValue(t));
}

double Container::ValueOfVy(double t){
	return(V.VyValue(t));
}

double Container::ValueOfVz(double t){
	return(V.VzValue(t));
}

double Container::ValueOfWx(double t){
	return(V.WxValue(t));
}

double Container::ValueOfWy(double t){
	return(V.WyValue(t));
}

double Container::ValueOfWz(double t){
	return(V.WzValue(t));
}

void Container::setMass(double m){
	Mass = m;
}


void Container::initTimeStep(){
	Fx = 0.;
	Fy = 0.;
	Fz = 0.;
	Mx = 0.;
	My = 0.;
	Mz = 0.;
}


void Container::setMemoryPosition(){
	xMemory = x;
	yMemory = y;
	zMemory = z;
}

void Container::getMemoryPosition(){
	x = xMemory;
	y = yMemory;
	z = zMemory;
}

void Container::upDateForce(){
	if(Fcx == 0 && Fcy == 0 && Fcz == 0 && Mcx == 0 && Mcy == 0 && Mcz == 0 && activeGravity == 0)
		Force = 0;
	else
		Force = 1;
}


void Container::setFcx(double fx){
	Fcx = fx;
}

void Container::setFcy(double fy){
	Fcy = fy;
}

void Container::setFcz(double fz){
	Fcz = fz;
}

double Container::getFcy(){
	return Fcy;
}

