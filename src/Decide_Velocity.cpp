#include "Decide_Velocity.h"

void Decide_Velocity::Decide_targetvelocity() {
	car.canditate_velocity = std::vector<int>(constants.N, 0);
	for (int ID = 0; ID < constants.N; ++ID) {
		int v = car.velocity.current[ID];
		car.S[ID] = constants.S;
		v = _rule1(ID, v);
		v = _rule2(ID, v);
		v = _rule3(ID, v);
		car.canditate_velocity[ID] = _rule4(ID, v);
	}
}

int Decide_Velocity::_rule1(int ID, int v) {
	int VelocityDifference = car.velocity.current[car.around.preceding.current[ID]] - car.velocity.current[ID];
	bool isPrecedingCarFasterorSame = false;
	if (VelocityDifference >= 0) isPrecedingCarFasterorSame = true;
	if (car.headway.current[ID] >= constants.G || isPrecedingCarFasterorSame == true) {
		v = std::min(car.Vmax[ID], v + 1);
	}
	return v;
}

int Decide_Velocity::_rule2(int ID, int v) {
	if (random->random(1.0) <= constants.q) {
		if (random->random(1.0) > constants.r) car.S[ID] = 1;
		int previous_preceedingcarID = car.around.preceding.previous[ID];
		int s = 1;
		for (s; s < car.S[ID]; s++) previous_preceedingcarID = car.around.preceding.previous[previous_preceedingcarID];
		int distance = car.position.previous[previous_preceedingcarID] - car.position.previous[ID];
		distance -= car.S[ID];
		if (distance < 0) distance += constants.lanelength;
		v = std::min(v, distance);
	}
	if (v < 0) {
		std::cout << "error V2 " << ID << std::endl;
		std::cout << v << std::endl;
		std::getchar();
	}
	return v;
}

int Decide_Velocity::_rule3(int ID, int v) {
		if (random->random(1.0) > constants.r) car.S[ID] = 1;
		int preceedingcarID = car.around.preceding.current[ID];
		int s = 1;
		for (s; s < car.S[ID]; s++) preceedingcarID = car.around.preceding.current[preceedingcarID];
		int distance = car.position.current[preceedingcarID] - car.position.current[ID];
		distance -= car.S[ID];
		if (distance < 0) distance += constants.lanelength;
		v = std::min(v, distance);
		//std::cout << ID << ":" << v << "," << distance << std::endl;
	if (v < 0) {
		std::cout << "error V3 " << ID << std::endl;
		std::cout << v << std::endl;
		std::getchar();
	}
	return v;
}

int Decide_Velocity::_rule4(int ID, int v) {
	p = constants.p1;
	int preceedingcarID = car.around.preceding.current[ID];
	int VelocityDifference = car.velocity.current[preceedingcarID] - car.velocity.current[ID];
	if (car.headway.current[ID] < constants.G) {
		if (VelocityDifference > 0) p = constants.p2;
		if (VelocityDifference == 0) p = constants.p3;
		if (VelocityDifference < 0) p = constants.p4;
	}
	if (random->random(1.0) < 1 - p) {
		if (v == 0) v = 0;
		else if (v - 1 > 1) v -= 1;
		else v = 1;
	}
	if (v < 0) {
		std::cout << "error V4 " << ID << std::endl;
		std::cout << v << std::endl;
		std::getchar();
	}
	return v;
}
