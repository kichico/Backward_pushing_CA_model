#include "Map_information.h"

void Map_information::Initialize(int Numberoflane, int lanelength) {
	recorded.existence.current = recorded.existence.previous = std::vector<std::vector<bool> >(Numberoflane,std::vector<bool>(lanelength, false));
	recorded.ID.current = recorded.ID.previous = std::vector<std::vector<int> >(Numberoflane, std::vector<int>(lanelength, -1));
	eachlanevehicle = std::vector<int>(Numberoflane, 0);
	lanevelocity = 0;
}

void Map_information::Fromcurrent_toprevious(){
	recorded.existence.previous = recorded.existence.current;
	recorded.ID.previous = recorded.ID.current;
}
