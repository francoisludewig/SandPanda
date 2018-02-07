#include "../Includes/MultiThread.h"

MultiThread::MultiThread(int threadCount, int sphereCount, int bodyCount, std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb, Configuration & dat, Gravity & gf, const CellBounds& cellBounds) {
	/*
	for(int i = 0 ; i < threadCount ; ++i) {
		evolutions.push_back(Evolution(sphereCount, bodyCount, pl, plr, co, elb, dat, gf, cellBounds, true));
	}
	*/
}

void MultiThread::Run(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Configuration & dat,Gravity & gf,
		std::vector<Sphere*>& cell, int & Ntp, char *name, int Nthreshold) {
	/*
	for(int i = 0 ; i < evolutions.size() ; ++i) {
		threads.push_back(std::thread(&Evolution::Evolve, &evolutions[i], std::ref(pl),std::ref(plr),std::ref(co),std::ref(elb),std::ref(sph),std::ref(bd),std::ref(hb),dat,gf,std::ref(cell),Ntp, name,Nthreshold));
	}

	for(auto& thread : threads)
		thread.join();
		*/
}
