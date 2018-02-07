#pragma once

#include "Elongation.h"
#include "../Contact/Contact.h"

#include <memory>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <mutex>

class ElongationManager {
public:

	class ElongationData {
   	public:
		ElongationData() = default;
		~ElongationData() noexcept {}

		ElongationData(int maxElongationCount) noexcept {
				elongations.resize(maxElongationCount);
				neighbourNumber.resize(maxElongationCount);
				neighbourCount = 0;
				type.resize(maxElongationCount);
				bodyNumber.resize(maxElongationCount);
				selfBodyNumber.resize(maxElongationCount);
		}

		ElongationData(const ElongationData& other) = default;
		ElongationData(ElongationData&& other) noexcept = default;
		ElongationData& operator=(const ElongationData& other) = default;
		ElongationData& operator=(ElongationData&& other) noexcept = default;

		void Reset() { neighbourCount = 0; }

		void WriteToFileForSphere(FILE *ft) const {
			fprintf(ft,"%d\n",neighbourCount);
				for(int i = 0 ; i < neighbourCount ; i++)
					fprintf(ft,"%d\t%d\t%e\t%e\t%e\n",
							neighbourNumber[i],
							static_cast<int>(type[i]),
							elongations[i].x,
							elongations[i].y,
							elongations[i].z);

		}

	    void WriteToFileForBody(FILE *ft) const {
	    	fprintf(ft,"%d\n",neighbourCount);
	    		for(int i = 0 ; i < neighbourCount ; i++)
	    			fprintf(ft,"%d\t%d\t%d\t%d\t%e\t%e\t%e\n",neighbourNumber[i],type[i],selfBodyNumber[i],bodyNumber[i],elongations[i].x,elongations[i].y,elongations[i].z);
	    }

		void ReadFromFileForSphere(FILE *ft) {
			fscanf(ft,"%d\n",&neighbourCount);
			for(int i = 0 ; i < neighbourCount ; i++)
				fscanf(ft,"%d\t%d\t%lf\t%lf\t%lf\n",&neighbourNumber[i],&type[i],&elongations[i].x,&elongations[i].y,&elongations[i].z);

		}

	    void ReadFromFileForBody(FILE *ft) {
	    	fscanf(ft,"%d\n",&neighbourCount);
	    		for(int i = 0 ; i < neighbourCount ; i++)
	    			fscanf(ft,"%d\t%d\t%d\t%d\t%lf\t%lf\t%lf\n",&neighbourNumber[i],&type[i],&selfBodyNumber[i],&bodyNumber[i],&elongations[i].x,&elongations[i].y,&elongations[i].z);
	    }

		std::vector<Elongation>  elongations;
	    std::vector<int>  neighbourNumber;
		int neighbourCount {0};
		std::vector<Contact::Type> type {Contact::Type::None};
		std::vector<int> bodyNumber;
		std::vector<int> selfBodyNumber;
	};


	ElongationManager() = default;

	ElongationManager(int maxElongationCount) {
		read = std::make_shared<ElongationData>(maxElongationCount);
		write = std::make_shared<ElongationData>(maxElongationCount);
		mutex = std::unique_ptr<std::mutex>(new std::mutex());
	}

	ElongationManager(const ElongationManager& other) = delete;
	ElongationManager(ElongationManager&& other) = default;
	ElongationManager& operator=(const ElongationManager& other) = delete;
	ElongationManager& operator=(ElongationManager&& other) = default;

	// Sphere - *
	void AddElongation(Elongation& elongation, int neighbourNumber , Contact::Type contactType, int bodyNumber) noexcept {
		write->elongations[write->neighbourCount] = elongation;
		write->neighbourNumber[write->neighbourCount] = neighbourNumber;
		write->type[write->neighbourCount] = contactType;
		write->bodyNumber[write->neighbourCount] = bodyNumber;
		write->neighbourCount++;
	}

	// Body - *
	void AddElongation(Elongation& elongation, int neighbourNumber , Contact::Type contactType, int selfBodyNumner, int bodyNumber) noexcept {
		write->elongations[write->neighbourCount] = elongation;
		write->neighbourNumber[write->neighbourCount] = neighbourNumber;
		write->type[write->neighbourCount] = contactType;
		write->bodyNumber[write->neighbourCount] = bodyNumber;
		write->selfBodyNumber[write->neighbourCount] = selfBodyNumner;
		write->neighbourCount++;
	}

	// Sphere - *
	void AddElongationMutex(Elongation& elongation, int neighbourNumber , Contact::Type contactType, int bodyNumber) noexcept {
		std::lock_guard<std::mutex> lock(*mutex);
		write->elongations[write->neighbourCount] = elongation;
		write->neighbourNumber[write->neighbourCount] = neighbourNumber;
		write->type[write->neighbourCount] = contactType;
		write->bodyNumber[write->neighbourCount] = bodyNumber;
		write->neighbourCount++;
	}
	
	// Body - *
	void AddElongationMutex(Elongation& elongation, int neighbourNumber , Contact::Type contactType, int selfBodyNumner, int bodyNumber) noexcept {
		std::lock_guard<std::mutex> lock(*mutex);
		write->elongations[write->neighbourCount] = elongation;
		write->neighbourNumber[write->neighbourCount] = neighbourNumber;
		write->type[write->neighbourCount] = contactType;
		write->bodyNumber[write->neighbourCount] = bodyNumber;
		write->selfBodyNumber[write->neighbourCount] = selfBodyNumner;
		write->neighbourCount++;
	}
	
	
	// Contact Sphere - *
	Elongation GetElongation(int neighbourNumber, Contact::Type contactType, int bodyNumber) const noexcept {
		for(int i = 0 ; i < read->neighbourCount ; i++){
			if(read->neighbourNumber[i] == neighbourNumber && read->type[i] == contactType){
				if(read->type[i] != Contact::Type::SphereBody /*10*/){
					return read->elongations[i];
				}
				else{
					if(bodyNumber == read->bodyNumber[i])
						return read->elongations[i];
				}
	        }
		}
		return Elongation::Empty();
	}

	// Contact Body - *
	Elongation GetElongation(int neighbourNumber, Contact::Type contactType,  int selfBodyNumber, int bodyNumber) const noexcept {
		for(int i = 0 ; i < read->neighbourCount ; i++){
			if(read->neighbourNumber[i] == neighbourNumber &&
					read->type[i] == contactType &&
					read->selfBodyNumber[i] == selfBodyNumber &&
					read->bodyNumber[i] == bodyNumber)
				return read->elongations[i];
		}
		return Elongation::Empty();
	}

	void InitXsi() noexcept {
		read.swap(write);
		write->Reset();
	}
	// TODO use write->
	void WriteToFileForSphere(FILE *ft) const { read->WriteToFileForSphere(ft); }
	// TODO use write->
	void WriteToFileForBody(FILE *ft) const { read->WriteToFileForBody(ft); }

    void ReadFromFileForSphere(FILE *ft) { read->ReadFromFileForSphere(ft); }
    void ReadFromFileForBody(FILE *ft) { read->ReadFromFileForBody(ft); }

private:
	std::unique_ptr<std::mutex> mutex {nullptr};
	std::shared_ptr<ElongationData> read;
	std::shared_ptr<ElongationData> write;

};
