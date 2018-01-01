#include "../Includes/Move.h"

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
#include "../Includes/ComputingForce.h"
#include "../Includes/Data.h"
#include "../Includes/Move.h"

void Move::UpDateForceContainer(int & Nsph, vector<Sphere> & sph, int & Npl, int & Nplr, int & Nco, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, double time, double dt, Gravity gt) noexcept {
	for(int i = 0 ; i < Npl ; i++){
		pl[i].UpdateForceFromGB(Nsph,sph);
	}
	for(int i = 0 ; i < Nplr ; i++){
		plr[i].UpdateForceFromGB(Nsph,sph);
	}
	for(int i = 0 ; i < Nco ; i++){
		co[i].UpdateForceFromGB(Nsph,sph);
		co[i].LimitForce();
	}
}

void Move::upDateVelocityContainer(int & Npl, int & Nplr, int & Nco, int & Nelb, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb, double time, double dt, Gravity gt) noexcept {
	for(int i = 0 ; i < Npl ; i++){
		pl[i].UpDateVelocity(time,dt,gt);
	}
	for(int i = 0 ; i < Nplr ; i++){
		plr[i].UpDateVelocity(time,dt,gt);
	}
	for(int i = 0 ; i < Nco ; i++){
		co[i].UpDateVelocity(time,dt,gt);
	}
}



void Move::moveContainer(int & Npl, int & Nplr, int & Nco, int & Nelb, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb, double time, double dt, vector<Sphere> & sph, Gravity gt) noexcept {
	for(int i = 0 ; i < Npl ; i++){
		pl[i].Move(dt);
		pl[i].UpDateLinkedSphere(sph,time,gt);
	}
	for(int i = 0 ; i < Nplr ; i++){
		plr[i].Move(dt);
		plr[i].UpDateLinkedSphere(sph,time,gt);
	}
	for(int i = 0 ; i < Nco ; i++){
		co[i].Move(dt);
		co[i].UpDateLinkedSphere(sph,time,gt);
		co[i].LimitUpdate();
	}
	for(int i = 0 ; i < Nelb ; i++){
		elb[i].Move(time,dt);
	}
}

void Move::upDateVelocityLinkedSphereContainer(int & Npl, int & Nplr, int & Nco, int & Nelb, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb, double time, double dt, vector<Sphere> & sph) noexcept {
	for(int i = 0 ; i < Npl ; i++){
		pl[i].UpDateVelocityLinkedSphere(sph,time);
	}
	for(int i = 0 ; i < Nplr ; i++){
		plr[i].UpDateVelocityLinkedSphere(sph,time);
	}
	for(int i = 0 ; i < Nco ; i++){
		co[i].UpDateVelocityLinkedSphere(sph,time);
	}
}

void Move::upDateVelocitySphere(int & Nsph, vector<Sphere> & sph, Gravity gt, double dt) noexcept {
	Sphere *sphl = &sph[0];
	// Frein viscqueux air
	for(int i = 0 ; i < Nsph ; i++){
		sphl->upDateVelocity(dt,gt,0.0);
		sphl++;
	}
}

void Move::upDateHollowBall(const int &Nhb, vector<HollowBall> & hb, double dt) noexcept {
	for(int i = 0 ; i < Nhb ; i++)
		hb[i].UpdateFromSph(dt);
		}

void Move::MeltingSphere(int & Nsph, vector<Sphere> & sph, double vr, double delayVr, double dt) noexcept {
	for(int i = 0 ; i < Nsph ; i++){
		sph[i].Freeze(vr/delayVr, dt);
	}
}

void Move::upDateVelocitySphereOMP(int & Nsph, vector<Sphere> & sph, Gravity gt, double dt, int Nprocess) noexcept {
	if(Nprocess == 2){
#pragma omp parallel 
		{
#pragma omp sections 
			{
#pragma omp section				
			for(int i = 0 ; i < Nsph/2 ; i++){
				sph[i].upDateVelocity(dt,gt,0.0);
			}
#pragma omp section				
			for(int i = Nsph/2 ; i < Nsph ; i++){
				sph[i].upDateVelocity(dt,gt,0.0);
			}
			}
		}
	}
	else{
#pragma omp parallel 
		{
#pragma omp sections 
			{
#pragma omp section				
			for(int i = 0 ; i < Nsph/4 ; i++){
				sph[i].upDateVelocity(dt,gt,0.0);
			}
#pragma omp section				
			for(int i = Nsph/4 ; i < Nsph/2 ; i++){
				sph[i].upDateVelocity(dt,gt,0.0);
			}
#pragma omp section				
			for(int i = Nsph/2 ; i < 3*Nsph/4 ; i++){
				sph[i].upDateVelocity(dt,gt,0.0);
			}
#pragma omp section				
			for(int i = 3*Nsph/4 ; i < Nsph ; i++){
				sph[i].upDateVelocity(dt,gt,0.0);
			}
			}
		}
		
	}
}

void Move::moveSphere(int & Nsph, vector<Sphere> & sph, double dt) noexcept {
	Sphere *sphl = &sph[0];
	for(int i = 0 ; i < Nsph ; i++){
		sphl->move(dt);
		sphl++;
	}
}

void Move::moveSphereOMP(int & Nsph, vector<Sphere> & sph, double dt, int Nprocess) noexcept {
	if(Nprocess == 2){
#pragma omp parallel 
		{
#pragma omp sections 
			{
#pragma omp section	
			for(int i = 0 ; i < Nsph/2 ; i++){
				sph[i].move(dt);
			}
#pragma omp section
			for(int i = Nsph/2 ; i < Nsph ; i++){
				sph[i].move(dt);
			}
			}
		}
	}
	else{
		
#pragma omp parallel 
		{
#pragma omp sections 
			{
#pragma omp section	
			for(int i = 0 ; i < Nsph/4 ; i++){
				sph[i].move(dt);
			}
#pragma omp section
			for(int i = Nsph/4 ; i < Nsph/2 ; i++){
				sph[i].move(dt);
			}
#pragma omp section
			for(int i = Nsph/2 ; i < 3*Nsph/4 ; i++){
				sph[i].move(dt);
			}
			
#pragma omp section
			for(int i = 3*Nsph/4 ; i < Nsph ; i++){
				sph[i].move(dt);
			}
			
			}
		}
		
	}
}

void Move::moveBodies(int & Nbd, vector<Body> & bd, double dt, vector<Sphere> & sph) noexcept {
	for(int i = 0 ; i < Nbd ; i++){
		bd[i].Move(dt);
		bd[i].UpDateLinkedSphere(sph);
	}
}

void Move::moveBodiesOMP(int & Nbd, vector<Body> & bd, double dt, vector<Sphere> & sph, int Nprocess) noexcept {
	if(Nprocess == 2){
#pragma omp parallel 
		{
#pragma omp sections 
			{
#pragma omp section	
			for(int i = 0 ; i < Nbd/2 ; i++){
				bd[i].Move(dt);
				bd[i].UpDateLinkedSphere(sph);
			}
			
#pragma omp section	
			for(int i = Nbd/2 ; i < Nbd ; i++){
				bd[i].Move(dt);
				bd[i].UpDateLinkedSphere(sph);
			}
			}
		}
	}
	else{
#pragma omp parallel 
		{
#pragma omp sections 
			{
#pragma omp section	
			for(int i = 0 ; i < Nbd/4 ; i++){
				bd[i].Move(dt);
				bd[i].UpDateLinkedSphere(sph);
			}
			
#pragma omp section	
			for(int i = Nbd/4 ; i < Nbd/2 ; i++){
				bd[i].Move(dt);
				bd[i].UpDateLinkedSphere(sph);
			}
			
#pragma omp section	
			for(int i = Nbd/2 ; i < 3*Nbd/4 ; i++){
				bd[i].Move(dt);
				bd[i].UpDateLinkedSphere(sph);
			}
			
#pragma omp section	
			for(int i = 3*Nbd/4 ; i < Nbd ; i++){
				bd[i].Move(dt);
				bd[i].UpDateLinkedSphere(sph);
			}
			
			}
		}
	}
}


void Move::upDateVelocityBodies(int & Nbd, vector<Body> & bd, Gravity gt, double dt, vector<Sphere> & sph) noexcept {
	for(int i = 0 ; i < Nbd ; i++){
		bd[i].UpDateVelocity(dt,gt);
		bd[i].UpDateLinkedSphere(sph);
	}
}

void Move::upDateVelocityBodiesOMP(int & Nbd, vector<Body> & bd, Gravity gt, double dt, vector<Sphere> & sph, int Nprocess) noexcept {
	if(Nprocess == 2){
#pragma omp parallel 
		{
#pragma omp sections 
			{
#pragma omp section	
			for(int i = 0 ; i < Nbd/2 ; i++){
				bd[i].UpDateVelocity(dt,gt);
				bd[i].UpDateLinkedSphere(sph);
			}
#pragma omp section	
			for(int i = Nbd/2 ; i < Nbd ; i++){
				bd[i].UpDateVelocity(dt,gt);
				bd[i].UpDateLinkedSphere(sph);
			}
			}
		}
	}
	else{
#pragma omp parallel 
		{
#pragma omp sections 
			{
#pragma omp section	
			for(int i = 0 ; i < Nbd/4 ; i++){
				bd[i].UpDateVelocity(dt,gt);
				bd[i].UpDateLinkedSphere(sph);
			}
#pragma omp section	
			for(int i = Nbd/4 ; i < Nbd/2 ; i++){
				bd[i].UpDateVelocity(dt,gt);
				bd[i].UpDateLinkedSphere(sph);
			}
#pragma omp section	
			for(int i = Nbd/2 ; i < 3*Nbd/4 ; i++){
				bd[i].UpDateVelocity(dt,gt);
				bd[i].UpDateLinkedSphere(sph);
			}
#pragma omp section	
			for(int i = 3*Nbd/4 ; i < Nbd ; i++){
				bd[i].UpDateVelocity(dt,gt);
				bd[i].UpDateLinkedSphere(sph);
			}
			}
		}
	}
}
