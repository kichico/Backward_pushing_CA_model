#include "Initialize.h"

void Initialize::InitializeEachSettings(int Numberofvehicle, int Numberoflane, int lanelength, int Cooperator) {
	isCorrelated = false;
	isDistributed = false;
	Defectoreachlane = std::vector<int>(Numberoflane, 0);
	this->Cooperator = Cooperator;
	this->Defector = constants.N - Cooperator;
	car.Initialize(constants.N);
	map.Initialize(constants.Numberoflane, constants.lanelength);
	_InitialPlacement(Numberoflane,lanelength);
	_AssignStrategy();
	_AssignVmax();
	car.around.following.previous = car.around.following.current;
	car.around.preceding.previous = car.around.preceding.current;
}

void Initialize::_InitialPlacement(int Numberoflane,int lanelength) {
	int AllCell = (int)(Numberoflane * lanelength);
	std::vector<int> availablecells(AllCell);
	car.leadingvehicle = std::vector<Car_information::LeadingVehicle>(Numberoflane);
	for (int i = 0; i < AllCell; ++i) availablecells[i] = i;
	for (int ID = 0; ID < constants.N; ++ID) {
		int picker = random->random((int)availablecells.size() - 1);
		int position = availablecells[picker] % lanelength;
		int lanenumber = availablecells[picker] / lanelength;
		std::iter_swap(availablecells.begin() + picker, availablecells.end() - 1);
		availablecells.pop_back();
		//the calculation speed of "iter_swap + pop_back" method is faster than "erase" method
		car.position.current[ID] = car.position.previous[ID] = position;
		car.lanenumber.current[ID] = car.lanenumber.previous[ID] = lanenumber;
		map.recorded.existence.current[lanenumber][position] = map.recorded.existence.previous[lanenumber][position] = true;
		map.recorded.ID.current[lanenumber][position] = map.recorded.ID.previous[lanenumber][position] = ID;
		map.eachlanevehicle[car.lanenumber.current[ID]]++;
		if (!car.leadingvehicle[lanenumber].existence) {
			car.leadingvehicle[lanenumber].ID = ID;
			car.leadingvehicle[lanenumber].existence = true;
		}
	}
	for (int lanenumber = 0; lanenumber < Numberoflane; ++lanenumber) {
		if (!car.leadingvehicle[lanenumber].existence) continue;
		if (map.eachlanevehicle[lanenumber] == 1) {
			int focalcarID = car.leadingvehicle[lanenumber].ID;
			car.around.following.current[focalcarID] = car.around.preceding.current[focalcarID] = focalcarID;
			continue;
		}
		carID IDs;
		IDs.detectedcarID = car.leadingvehicle[lanenumber].ID;
		int firstdetected = IDs.detectedcarID;
		int start = car.position.current[IDs.detectedcarID];
		int currentposition = start + 1;
		int distance = 0;
		while (true) {
			if (distance >= lanelength) distance -= lanelength;
			if (currentposition >= lanelength) currentposition -= lanelength;
			if (map.recorded.existence.current[lanenumber][currentposition]) {
				IDs.foundcarID = map.recorded.ID.current[lanenumber][currentposition];
				_FillEachInfomation(IDs, distance);
				if (IDs.foundcarID == firstdetected) {
					car.around.following.current[IDs.foundcarID] = IDs.detectedcarID;
					break;
				}
				distance = 0;
				IDs.detectedcarID = IDs.foundcarID;
			}
			else ++distance;
			++currentposition;
		}
	}
}

void Initialize::_FillEachInfomation(carID IDs, int distance) {
	int lanenumber = car.lanenumber.current[IDs.foundcarID];
	car.around.preceding.current[IDs.detectedcarID] = IDs.foundcarID;
	car.around.following.current[IDs.foundcarID] = IDs.detectedcarID;
	car.headway.current[IDs.detectedcarID] = distance;
	if (distance > Maxheadway) {
		car.leadingvehicle[lanenumber].ID = IDs.detectedcarID;
		car.leadingvehicle[lanenumber].distance = distance;
		Maxheadway = distance;
	}
}

void Initialize::_AssignStrategy() {
	std::vector<int> DCar(constants.N),CCar;
	for (int i = 0; i < constants.N; ++i) DCar[i] = i;
	for (int i = 0; i < Cooperator; ++i) {
		int Itwillbecooperator = (int)DCar.size() - 1;
		int picker = random->random(Itwillbecooperator);
		car.strategy[DCar[picker]] = Car_information::StrategyKind::C;
		CCar.emplace_back(DCar[picker]);
		std::iter_swap(DCar.begin() + picker, DCar.end() - 1);
		DCar.pop_back();
	}
	for (int i = 0; i < DCar.size(); ++i) car.strategy[DCar[i]] = Car_information::StrategyKind::D;
	car.List_Defector = DCar;
	car.List_Cooperator = CCar;
}

void Initialize::_AssignVmax() {
	if (isDistributed == false && isCorrelated == false) {
		for (int i = 0; i < constants.N; ++i) car.Vmax[i] = 5;
	}
	else if (isDistributed == true && isCorrelated == false) {
		std::vector<int> NotassignedVmax(constants.N);
		for (int i = 0; i < constants.N; ++i) NotassignedVmax[i] = i;
		for (int i = 0; i < constants.N; ++i) {
			int Numofnotassigned = (int)NotassignedVmax.size() - 1;
			int picker = random->random(Numofnotassigned);
			//std::cout << "picked:" << NotassignedVmax[picker] << std::endl;
			if (i % 3 == 0) car.Vmax[NotassignedVmax[picker]] = 4;
			else if (i % 3 == 1) car.Vmax[NotassignedVmax[picker]] = 5;
			else car.Vmax[NotassignedVmax[picker]] = 6;
			std::iter_swap(NotassignedVmax.begin() + picker, NotassignedVmax.end() - 1);
			NotassignedVmax.pop_back();
		}
	}
	else if (isDistributed == true && isCorrelated == true) {
		int slowspeedcar = (int)std::round((double)constants.N / 3);
		int highspeedcar = constants.N - 2 * slowspeedcar;
		int generalspeedcar = constants.N - slowspeedcar - highspeedcar;
		int alreadyassigned = 0;
		for (int i = 0; i < (int)car.List_Defector.size(); ++i) {
			if (alreadyassigned < highspeedcar) car.Vmax[car.List_Defector[i]] = 6;
			else if (alreadyassigned < (highspeedcar + generalspeedcar)) car.Vmax[car.List_Defector[i]] = 5;
			else car.Vmax[car.List_Defector[i]] = 4;
			alreadyassigned++;
		}
		for (int i = 0; i < (int)car.List_Cooperator.size(); ++i) {
			if (alreadyassigned < highspeedcar) car.Vmax[car.List_Cooperator[i]] = 6;
			else if (alreadyassigned < (highspeedcar + generalspeedcar)) car.Vmax[car.List_Cooperator[i]] = 5;
			else car.Vmax[car.List_Cooperator[i]] = 4;
			alreadyassigned++;
		}
	}
	else {
		std::cout << "You did wrong settings at distributed region" << std::endl;
		exit(0);
	}
}
