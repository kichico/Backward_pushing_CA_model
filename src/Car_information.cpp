#include "Car_information.h"

void Car_information::Initialize(int Numberofvehicle) {
	velocity.current = velocity.previous = canditate_velocity = std::vector<int>(Numberofvehicle, 0);
	position.current = position.previous = std::vector<int>(Numberofvehicle, 0);
	headway.current = headway.previous = std::vector<int>(Numberofvehicle, 0);
	around.following.current = around.following.previous = std::vector<int>(Numberofvehicle, 0);
	around.preceding.current = around.preceding.previous = std::vector<int>(Numberofvehicle, 0);
	lanenumber.current = lanenumber.previous = std::vector<int>(Numberofvehicle, 0);
	signal = std::vector<int>(Numberofvehicle, 0);
	strategy = canditate_velocity = Vmax = std::vector<int>(Numberofvehicle, 0);
	S = std::vector<int>(Numberofvehicle, 2);
	pushing.isPushing = std::vector<bool>(Numberofvehicle, false);
	pushing.Preceding = std::vector<int>(Numberofvehicle, -1);
}

void Car_information::Fromcurrent_toprevious() {
	velocity.previous = velocity.current;
	position.previous = position.current;
	headway.previous = headway.current;
	lanenumber.previous = lanenumber.current;
	around.following.previous = around.following.current;
	around.preceding.previous = around.preceding.current;
}