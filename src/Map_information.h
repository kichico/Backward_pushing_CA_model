#pragma once
#include <vector>

class Map_information {
	struct CurrentandPreviousbool {
		std::vector<std::vector<bool> > current;
		std::vector<std::vector<bool> > previous;
	};
	struct CurrentandPreviousint {
		std::vector<std::vector<int> > current;
		std::vector<std::vector<int> > previous;
	};
	struct map_information {
		CurrentandPreviousbool existence;
		CurrentandPreviousint ID;
	};
public:
	struct update_information {
		std::vector<std::vector<bool> > existence;
		std::vector<std::vector<int> > ID;
	};
	int lanevelocity;
	std::vector<int> eachlanevehicle;
	map_information recorded;
	void Initialize(int Numberoflane,int lanelength);
	void Fromcurrent_toprevious();
};