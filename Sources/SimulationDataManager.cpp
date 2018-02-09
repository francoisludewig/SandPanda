#include "../Includes/SimulationDataManager.h"
#include "../Includes/Solids/SimulationData.h"
#include "../Includes/ReadWrite.h"
#include "../Includes/Solids/Body.h"
#include "../Includes/Option.h"

std::shared_ptr<SimulationData> SimulationDataManager::FromExport(Option& opt) noexcept {
	auto solids = std::shared_ptr<SimulationData>(new SimulationData());
	ReadWrite::readOutContainer(opt.directory, solids->plans, solids->disks, solids->cones, solids->elbows);
	ReadWrite::readOutSphere(opt.directory, solids->spheres , opt.limitNg);
	ReadWrite::readOutBodies(opt.directory, solids->bodies, opt.limitNbd);

	solids->sphereCount = solids->spheres.size();
	LoadBodySpecies(opt, solids);

	ReadWrite::readOutData(opt.directory, solids->gravity, solids->configuration);
	ReadWrite::readOutHollowBall(opt.directory, solids->hollowBalls);

	return solids;
}

std::shared_ptr<SimulationData> SimulationDataManager::FromStart_Stop(Option& opt) noexcept {
	auto solids = std::shared_ptr<SimulationData>(new SimulationData());
	ReadWrite::readStart_stopContainer(opt.directory, solids->plans, solids->disks, solids->cones, solids->elbows);
	ReadWrite::readStart_stopSphere(opt.directory, solids->spheres, opt.limitNg);
	ReadWrite::readStart_stopBodies(opt.directory, solids->bodies, opt.limitNbd);

	solids->sphereCount = solids->spheres.size();
	LoadBodySpecies(opt, solids);

	ReadWrite::readStartStopData(opt.directory, solids->gravity, solids->configuration);
	ReadWrite::readStart_stopHollowBall(opt.directory, solids->hollowBalls);

	return solids;
}

void SimulationDataManager::LoadBodySpecies(Option &opt, std::shared_ptr<SimulationData>& solids) noexcept {
	ReadWrite::readOutBodySpecie(opt.directory, solids->bodySpecies);
	for( int i = 0 ; i < solids->bodies.size() ; i++)
		solids->bodies[i].UploadSpecies(solids->bodySpecies,solids->spheres,i);
}

void SimulationDataManager::WriteStart_Stop(Option &opt, std::shared_ptr<SimulationData>& solids, int ntp) noexcept {
	ReadWrite::writeStartStopContainer(opt.directory,solids->plans,solids->disks,solids->cones,solids->elbows);
	ReadWrite::writeStartStopSphere(opt.directory,solids->spheres);
	ReadWrite::writeStartStopBodies(opt.directory,solids->bodies,solids->spheres);
	ReadWrite::writeStartStopData(opt.directory, solids->gravity, solids->configuration);
	ReadWrite::writeStartStopHollowBall(opt.directory, solids->hollowBalls);

	ReadWrite::writeOutContainer(opt.directory,ntp,solids->plans,solids->disks,solids->cones,solids->elbows,solids->configuration.outMode);
	ReadWrite::writeOutSphere(opt.directory,ntp, solids->spheres,solids->configuration.outMode);
	ReadWrite::writeOutBodies(opt.directory,ntp,solids->bodies,solids->configuration.outMode);
	ReadWrite::writeOutData(opt.directory, ntp, solids->gravity, solids->configuration);
	ReadWrite::writeOutHollowBall(opt.directory, ntp, solids->hollowBalls);
}


