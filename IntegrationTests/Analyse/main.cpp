#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>

class Coherent {
public:
	Coherent(std::vector<std::string>& nameFile) {
		for(const auto& name : nameFile) {
			std::ifstream diffFile(name+"Diff.txt");
			isCoherent[name] = Is_empty(diffFile);
		}
	}
	
	bool IsCoherent(std::string name) { return isCoherent[name]; }
	
private:
	std::map<std::string,bool> isCoherent;
	
	bool Is_empty(std::ifstream& pFile) {
		return pFile.peek() == std::ifstream::traits_type::eof();
	}
};


class Efficiency {
public:
	Efficiency(std::vector<std::string>& nameFile) {
		for(const auto& name : nameFile) {
			std::ifstream dataFile(name+"New.txt");
			std::string data = getLastLine(dataFile);
			current[name] = std::stol(data.substr(6, data.size()-3-6));
		}
		for(const auto& name : nameFile) {
			std::ifstream dataFile(name+"Original.txt");
			std::string data = getLastLine(dataFile);
			ref[name] = std::stol(data.substr(6, data.size()-3-6));
		}
	}
	
	double GetRelativePercentage(const std::string& name) {
		return (100*(current[name] - ref[name]))/ref[name];
	}
	
private:
	std::map<std::string, int64_t> ref;
	std::map<std::string, int64_t> current;
	
	std::string getLastLine(std::ifstream& in) {
		std::string line;
		while (in >> std::ws && std::getline(in, line)); // skip empty line
		return line;
	}
};

class Report{
public:
	static void MakeReport(std::vector<std::string>& nameFile) {
		Coherent coherent(nameFile);
		Efficiency efficiency(nameFile);
		std::ofstream report("TestReport.txt");
		for(const auto& name : nameFile) {
			report << "------------------------------------" << std::endl;
			report << name.substr(3, name.size()-3) << std::endl;
			report << "Consistency test : " << (coherent.IsCoherent(name) ? "Succeed" : "Failed") << std::endl;
			report << "Efficiency test : " << efficiency.GetRelativePercentage(name) << "%" << std::endl;
		}
	}
};

int main(int argc,char **argv){
	std::vector<std::string> nameFile {
		"../BodyBox",
		"../MixBox",
		"../SphereBox",
		"../BodyCone",
		"../MixCone",
		"../SphereCone",
		"../MasterSolid"};
	Report::MakeReport(nameFile);
	
	return 0;
}