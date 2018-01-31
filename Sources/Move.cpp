#include "../Includes/Move.h"

#include "../Includes/Solids/Velocity.h"
#include "../Includes/Gravity.h"
#include "../Includes/Solids/Plan.h"
#include "../Includes/Solids/PlanR.h"
#include "../Includes/Solids/Cone.h"
#include "../Includes/Solids/Elbow.h"
#include "../Includes/Solids/Sphere.h"
#include "../Includes/Solids/Body.h"
#include "../Includes/Contact/Contact.h"
#include "../Includes/ReadWrite.h"
#include "../Includes/Contact/ContactDetection.h"
#include "../Includes/ComputingForce.h"
#include "../Includes/Data.h"
#include "../Includes/Move.h"
#include "../Includes/Solids/HollowBall.h"
#include "../Includes/LinkedCells/CellBounds.h"

void Move::UpDateForceContainer(std::vector<Sphere> & sph, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, double time, double dt, Gravity& gt) noexcept {
	for(auto& plan : pl)
		plan.UpdateForceFromGB(sph);

	for(auto& disk : plr)
		disk.UpdateForceFromGB(sph);

	for(auto& cone : co) {
		cone.UpdateForceFromGB(sph);
		cone.LimitForce();
	}
}

void Move::upDateVelocityContainer(std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, double time, double dt, Gravity& gt) noexcept {
	for(auto& plan : pl)
		plan.UpDateVelocity(time,dt,gt);

	for(auto& disk : plr)
		disk.UpDateVelocity(time,dt,gt);

	for(auto& cone : co)
		cone.UpDateVelocity(time,dt,gt);
}


void Move::moveContainer(std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, double time, double dt, std::vector<Sphere> & sph, Gravity& gt) noexcept {
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

void Move::upDateVelocitySphere(std::vector<Sphere> & sph, std::vector<Sphere*> & cell, const CellBounds& cellBounds, Gravity& gt, double dt) noexcept {
	int num;
	Sphere* sphere;
	for(int i = cellBounds.StartX() ; i < cellBounds.EndX() ; ++i) {
		for(int j = cellBounds.StartY() ; j < cellBounds.EndY() ; ++j) {
			for(int k = cellBounds.StartZ() ; k < cellBounds.EndZ() ; ++k) {
				num = cellBounds.Index(i, j, k);
				if((sphere = cell[num]) != nullptr) {
					do {
						if(sphere->NoBodies())
							sphere->upDateVelocity(dt,gt,0.0);
						else {
							(sphere->GetBody())->UpDateVelocity(dt,gt);
							(sphere->GetBody())->UpDateLinkedSphere(sph);
						}
					} while((sphere = sphere->TDL()) != nullptr);
				}
			}
		}
	}
}


void Move::upDateHollowBall(std::vector<HollowBall> & hb, double dt) noexcept {
	for(auto& hollowBall : hb)
		hollowBall.UpdateFromSph(dt);
}

void Move::MeltingSphere(std::vector<Sphere> & sph, double vr, double delayVr, double dt) noexcept {
	for(auto& sphere : sph)
		sphere.Freeze(vr/delayVr, dt);
}

void Move::moveSphere(std::vector<Sphere> & sph, double dt) noexcept {
	for(auto& sphere : sph)
		sphere.move(dt);
}

void Move::moveSphere(std::vector<Sphere*> & cell, const CellBounds& cellBounds, double dt) noexcept {
	int num;
	Sphere* sphere;
	for(int i = cellBounds.StartX() ; i < cellBounds.EndX() ; ++i) {
		for(int j = cellBounds.StartY() ; j < cellBounds.EndY() ; ++j) {
			for(int k = cellBounds.StartZ() ; k < cellBounds.EndZ() ; ++k) {
				num = cellBounds.Index(i, j, k);
				if((sphere = cell[num]) != nullptr) {
					do {
						if(sphere->NoBodies())
							sphere->move(dt);
						else
							(sphere->GetBody())->Move(dt);
					} while((sphere = sphere->TDL()) != nullptr);
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

void Move::upDateVelocityBodies(std::vector<Body> & bd, Gravity gt, double dt, std::vector<Sphere> & sph) noexcept {
	for(auto& body : bd){
		body.UpDateVelocity(dt,gt);
		body.UpDateLinkedSphere(sph);
	}
}

