#include "../../Includes/Object/Elbow.h"

Elbow::Elbow() noexcept {
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
	NCell = 0;	
	NCell2 = 0;
	r = 0.1;
	xi = -0.25;
	yi = 0;
	zi = 0;
	xf = 0;
	yf = 0;
	zf = 0.25;
	cx = 0;
	cy = 0;
	cz = 1;
}

void Elbow::ReadFromFile(FILE *ft) noexcept {
	fscanf(ft,"%lf\t%lf\t%lf\n",&xi,&yi,&zi);
	fscanf(ft,"%lf\t%lf\t%lf\n",&xf,&yf,&zf);		
	fscanf(ft,"%lf\t%lf\t%lf\n",&x,&y,&z);
	fscanf(ft,"%lf\t%lf\t%lf\n",&nx,&ny,&nz);
	fscanf(ft,"%lf\t%lf\t%lf\n",&tx,&ty,&tz);
	fscanf(ft,"%lf\t%lf\t%lf\n",&sx,&sy,&sz);
	V.LoadFromFile(ft);	
	fscanf(ft,"%lf\t%lf\t%lf\n",&R,&alpha,&r);	
}

void Elbow::WriteToFile(FILE *ft) const noexcept {
	fprintf(ft,"%e\t%e\t%e\n",xi,yi,zi);
	fprintf(ft,"%e\t%e\t%e\n",xf,yf,zf);		
	fprintf(ft,"%e\t%e\t%e\n",x,y,z);
	fprintf(ft,"%e\t%e\t%e\n",nx,ny,nz);
	fprintf(ft,"%e\t%e\t%e\n",tx,ty,tz);
	fprintf(ft,"%e\t%e\t%e\n",sx,sy,sz);
	V.WriteToFile(ft,0);	
	fprintf(ft,"%e\t%e\t%e\n",R,alpha,r);		
}

void Elbow::WriteOutFile(FILE *ft, int mode) const noexcept {
	if(mode == 0){
		fprintf(ft,"%e\t%e\t%e\n",xi,yi,zi);
		fprintf(ft,"%e\t%e\t%e\n",xf,yf,zf);		
		fprintf(ft,"%e\t%e\t%e\n",x,y,z);
		fprintf(ft,"%e\t%e\t%e\n",nx,ny,nz);
		fprintf(ft,"%e\t%e\t%e\n",tx,ty,tz);
		fprintf(ft,"%e\t%e\t%e\n",sx,sy,sz);
		V.WriteToFile(ft,mode);	
		fprintf(ft,"%e\t%e\t%e\n",R,alpha,r);	
	}
	else{
		fwrite(&xi, sizeof(double), 1, ft);
		fwrite(&yi, sizeof(double), 1, ft);
		fwrite(&zi, sizeof(double), 1, ft);
		fwrite(&xf, sizeof(double), 1, ft);
		fwrite(&yf, sizeof(double), 1, ft);
		fwrite(&zf, sizeof(double), 1, ft);
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
		V.WriteToFile(ft,mode);			
		fwrite(&R, sizeof(double), 1, ft);
		fwrite(&alpha, sizeof(double), 1, ft);
		fwrite(&r, sizeof(double), 1, ft);
	}
}

void Elbow::Move(double time, double dt) noexcept {
	double ox,oy,oz;
	double Rot[3][3];
	// Translation
	vx = V.ValueOfVx(time);	
	vy = V.ValueOfVy(time);	
	vz = V.ValueOfVz(time);	
	x += vx*dt;
	y += vy*dt;
	z += vz*dt;
	// Rotation
	wx = V.ValueOfWx(time)*dt;
	wy = V.ValueOfWy(time)*dt;
	wz = V.ValueOfWz(time)*dt;
	
	double W = sqrt(wx*wx+wy*wy+wz*wz);	
	double Ca = cos(W),Sa = sin(W);
	if(W != 0){
		wx = wx/W;
		wy = wy/W;
		wz = wz/W;
		
		//      | wx^2+Ca*(1-wx^2)       wx*wy*(1-Ca)-wz*Sa     wx*wz*(1-Ca)+wy*Sa | 
		//[R] = | wx*wy*(1-Ca)+wz*Sa     wy^2+Ca*(1-wy^2)       wy*wz*(1-Ca)-wx*Sa | 
		//      | wx*wz*(1-Ca)-wy*Sa     wy*wz*(1-Ca)+wx*Sa     wz^2+Ca*(1-wz^2)   | 
		
		Rot[0][0] = wx*wx+Ca*(1-wx*wx) ;
		Rot[0][1] = wx*wy*(1-Ca)-wz*Sa;
		Rot[0][2] = wx*wz*(1-Ca)+wy*Sa;
		
		Rot[1][0] = wx*wy*(1-Ca)+wz*Sa;
		Rot[1][1] = wy*wy+Ca*(1-wy*wy);
		Rot[1][2] = wy*wz*(1-Ca)-wx*Sa;
		
		Rot[2][0] = wx*wz*(1-Ca)-wy*Sa;
		Rot[2][1] = wy*wz*(1-Ca)+wx*Sa;
		Rot[2][2] = wz*wz+Ca*(1-wz*wz);
	}
	else{
		Rot[0][0] = 1;
		Rot[0][1] = 0;
		Rot[0][2] = 0;
		
		Rot[1][0] = 0;
		Rot[1][1] = 1;
		Rot[1][2] = 0;
		
		Rot[2][0] = 0;
		Rot[2][1] = 0;
		Rot[2][2] = 1;
	}	
	
	
	ox = (x-V.ox);
	oy = (y-V.oy);
	oz = (z-V.oz);
	
	x = (Rot[0][0]*ox + Rot[0][1]*oy + Rot[0][2]*oz) + V.ox;
	y = (Rot[1][0]*ox + Rot[1][1]*oy + Rot[1][2]*oz) + V.oy;
	z = (Rot[2][0]*ox + Rot[2][1]*oy + Rot[2][2]*oz) + V.oz;
	
	ox = nx;
	oy = ny;
	oz = nz;
	
	nx = Rot[0][0]*ox + Rot[0][1]*oy + Rot[0][2]*oz;
	ny = Rot[1][0]*ox + Rot[1][1]*oy + Rot[1][2]*oz;
	nz = Rot[2][0]*ox + Rot[2][1]*oy + Rot[2][2]*oz;
	
	ox = tx;
	oy = ty;
	oz = tz;
	
	tx = Rot[0][0]*ox + Rot[0][1]*oy + Rot[0][2]*oz;
	ty = Rot[1][0]*ox + Rot[1][1]*oy + Rot[1][2]*oz;
	tz = Rot[2][0]*ox + Rot[2][1]*oy + Rot[2][2]*oz;
	
	
	ox = sx;
	oy = sy;
	oz = sz;
	
	sx = Rot[0][0]*ox + Rot[0][1]*oy + Rot[0][2]*oz;
	sy = Rot[1][0]*ox + Rot[1][1]*oy + Rot[1][2]*oz;
	sz = Rot[2][0]*ox + Rot[2][1]*oy + Rot[2][2]*oz;	
}


double Elbow::Vmax() const noexcept {
	return(V.VMax());
}

double Elbow::Wmax() const noexcept{
	return(V.WMax());
}

double Elbow::Delay() const noexcept{
	return(V.Delay());
}

