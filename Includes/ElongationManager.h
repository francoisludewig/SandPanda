#pragma once

#include "Elongation.h"

#include <memory>

class ElongationManager {
public:

	class ElongationMetadata {
		ElongationMetadata(int maxElongationCount) {
				xsi->resize(maxElongationCount);
				neighbourNumber->resize(maxElongationCount);
				neighbourCount = 0;
				type->resize(maxElongationCount);
				bodyNumber->resize(maxElongationCount);
		}

   	private:
		std::vector<Elongation>  xsi;
	    std::vector<int>  neighbourNumber;
		int neighbourCount;
		std::vector<int> type;
		std::vector<int> bodyNumber;

	};


	ElongationManager(int maxElongationCount) {
		read = std::make_unique(ElongationMetadata(maxElongationCount));
		write = std::make_unique(ElongationMetadata(maxElongationCount));
	}



private:
	std::unique_ptr<ElongationMetadata> read;
	std::unique_ptr<ElongationMetadata> write;

};
