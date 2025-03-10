#include "../../Includes/Object/Velocity.h"

Velocity::Velocity() noexcept : ox(0), oy(0), oz(0){}

void Velocity::LoadFromFile(FILE *ft) noexcept {
	vx.LoadFromFile(ft);
	vy.LoadFromFile(ft);	
	vz.LoadFromFile(ft);
	wx.LoadFromFile(ft);
	wy.LoadFromFile(ft);
	wz.LoadFromFile(ft);
	fscanf(ft,"%lf\t%lf\t%lf\n",&ox,&oy,&oz);
}

void Velocity::WriteToFile(FILE *ft, int mode) const noexcept {
	vx.WriteToFile(ft,mode);
	vy.WriteToFile(ft,mode);
	vz.WriteToFile(ft,mode);
	wx.WriteToFile(ft,mode);
	wy.WriteToFile(ft,mode);
	wz.WriteToFile(ft,mode);	
	if(mode == 0){
		fprintf(ft,"%e\t%e\t%e\n",ox,oy,oz);
	}
	else{
		fwrite(&ox, sizeof(double), 1, ft);
		fwrite(&oy, sizeof(double), 1, ft);
		fwrite(&oz, sizeof(double), 1, ft);
	}
}

void Velocity::LoadFromFileWithoutOrigin(FILE *ft) noexcept {
	vx.LoadFromFile(ft);
	vy.LoadFromFile(ft);	
	vz.LoadFromFile(ft);
	wx.LoadFromFile(ft);
	wy.LoadFromFile(ft);	
	wz.LoadFromFile(ft);
}

void Velocity::WriteToFileWithoutOrigin(FILE *ft, int mode) const noexcept {
	vx.WriteToFile(ft,mode);
	vy.WriteToFile(ft,mode);
	vz.WriteToFile(ft,mode);
	wx.WriteToFile(ft,mode);
	wy.WriteToFile(ft,mode);
	wz.WriteToFile(ft,mode);		
}

double Velocity::ValueOfVx(double t) const noexcept {
	return vx.Value(t);
}

double Velocity::ValueOfVy(double t) const noexcept {
	return vy.Value(t);
}

double Velocity::ValueOfVz(double t) const noexcept {
	return vz.Value(t);
}

double Velocity::ValueOfWx(double t) const noexcept {
	return wx.Value(t);
}

double Velocity::ValueOfWy(double t) const noexcept {
	return wy.Value(t);
}

double Velocity::ValueOfWz(double t) const noexcept {
	return wz.Value(t);
}

double Velocity::VMax() const noexcept {
	double v = vx.Max();
	if(v < vy.Max())v = vy.Max();
	if(v < vz.Max())v = vz.Max();
	return(v);
}

double Velocity::WMax() const noexcept {
	double v = wx.Max();
	if(v < wy.Max())v = wy.Max();
	if(v < wz.Max())v = wz.Max();
	return(v);
}

double Velocity::Delay() const noexcept {
	double d = vx.DelayTr();
	if(d < vy.DelayTr())d = vy.DelayTr();
	if(d < vz.DelayTr())d = vz.DelayTr();
	if(d < wx.DelayRot())d = wx.DelayRot();
	if(d < wy.DelayRot())d = wy.DelayRot();
	if(d < wz.DelayRot())d = wz.DelayRot();
	return(d);
}

void Velocity::Display() const noexcept {
	vx.Display();
	vy.Display();	
	vz.Display();	
	wx.Display();	
	wy.Display();	
	wz.Display();
}

void Velocity::Set(VelocityType type, double a0, double a1, double w, double phi) noexcept {
	switch(type) {
		case VelocityType::vx:
			vx = Sinusoid(a0, a1, w, phi);
			break;
		case VelocityType::vy:
			vy = Sinusoid(a0, a1, w, phi);
			break;
		case VelocityType::vz:
			vz = Sinusoid(a0, a1, w, phi);
			break;
		case VelocityType::wx:
			wx = Sinusoid(a0, a1, w, phi);
			break;
		case VelocityType::wy:
			wy = Sinusoid(a0, a1, w, phi);
			break;
		case VelocityType::wz:
			wz = Sinusoid(a0, a1, w, phi);
			break;
	}
}

