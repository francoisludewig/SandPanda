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
#include "../Includes/HollowBall.h"

void Move::UpDateForceContainer(std::vector<Sphere> & sph, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, double time, double dt, Gravity gt) noexcept {
	for(auto& plan : pl)
		plan.UpdateForceFromGB(sph);

	for(auto& disk : plr)
		disk.UpdateForceFromGB(sph);

	for(auto& cone : co) {
		cone.UpdateForceFromGB(sph);
		cone.LimitForce();
	}
}

void Move::upDateVelocityContainer(std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, double time, double dt, Gravity gt) noexcept {
	for(auto& plan : pl)
		plan.UpDateVelocity(time,dt,gt);

	for(auto& disk : plr)
		disk.UpDateVelocity(time,dt,gt);

	for(auto& cone : co)
		cone.UpDateVelocity(time,dt,gt);
}


void Move::moveContainer(std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, double time, double dt, std::vector<Sphere> & sph, Gravity gt) noexcept {
	for(auto& plan : pl) {
		plan.Move(dt);
		plan.UpDateLinkedSphere(sph,time,gt);
	}
	for(auto& disk : plr){
		disk.Move(dt);
		disk.UpDateLinkedSphere(sph,time,gt);
	}
	for(auto& cone : co){
		cone.Move(dt);
		cone.UpDateLinkedSphere(sph,time,gt);
		cone.LimitUpdate();
	}
	for(auto& elbow : elb)
		elbow.Move(time,dt);
}

void Move::upDateVelocityLinkedSphereContainer(std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, double time, double dt, std::vector<Sphere> & sph) noexcept {
	for(auto& plan : pl)
		plan.UpDateVelocityLinkedSphere(sph,time);

	for(auto& disk : plr)
		disk.UpDateVelocityLinkedSphere(sph,time);

	for(auto& cone : co)
		cone.UpDateVelocityLinkedSphere(sph,time);
}

void Move::upDateVelocitySphere(std::vector<Sphere> & sph, Gravity gt, double dt) noexcept {
	for(auto& sphere : sph)
		sphere.upDateVelocity(dt,gt,0.0);
}

void Move::upDateHollowBall(std::vector<HollowBall> & hb, double dt) noexcept {
	for(auto& hollowBall : hb)
		hollowBall.UpdateFromSph(dt);
}

void Move::MeltingSphere(std::vector<Sphere> & sph, double vr, double delayVr, double dt) noexcept {
	for(auto& sphere : sph)
		sphere.Freeze(vr/delayVr, dt);
}

void Move::upDateVelocitySphereOMP(int & Nsph, std::vector<Sphere> & sph, Gravity gt, double dt, int Nprocess) noexcept {
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

void Move::moveSphere(std::vector<Sphere> & sph, double dt) noexcept {
	for(auto& sphere : sph)
		sphere.move(dt);
}

void Move::moveSphereOMP(int & Nsph, std::vector<Sphere> & sph, double dt, int Nprocess) noexcept {
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

void Move::moveBodies(std::vector<Body> & bd, double dt, std::vector<Sphere> & sph) noexcept {
	for(auto& body : bd){
		body.Move(dt);
		body.UpDateLinkedSphere(sph);
	}
}

void Move::moveBodiesOMP(int & Nbd, std::vector<Body> & bd, double dt, std::vector<Sphere> & sph, int Nprocess) noexcept {
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


void Move::upDateVelocityBodies(std::vector<Body> & bd, Gravity gt, double dt, std::vector<Sphere> & sph) noexcept {
	for(auto& body : bd){
		body.UpDateVelocity(dt,gt);
		body.UpDateLinkedSphere(sph);
	}
}

void Move::upDateVelocityBodiesOMP(int & Nbd, std::vector<Body> & bd, Gravity gt, double dt, std::vector<Sphere> & sph, int Nprocess) noexcept {
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
