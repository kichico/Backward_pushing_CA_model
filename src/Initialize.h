#pragma once
#include "Car_information.h"
#include "Constants.h"
#include "Map_information.h"
#include "random.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <sstream>

class Initialize{
private:
	void _AssignVmax();
	void _InitialPlacement(int Numberoflane,int lanelength);
	void _AssignStrategy();
	struct carID {
		int foundcarID;
		int detectedcarID;
	};
	void _FillEachInfomation(carID IDs,int distance);
	int Maxheadway = 0;
public:
	struct Measuredinfomation {
		int Lanechange_original = 0;
		int Lanechange_Pushed = 0;
		int passed = 0;
		double average_velocity = 0.0;
		Measuredinfomation operator+=(const Measuredinfomation& other) {
			this->average_velocity += other.average_velocity;
			this->Lanechange_original += other.Lanechange_original;
			this->passed += other.passed;
			this->Lanechange_Pushed += other.Lanechange_Pushed;
			return *this;
		}
	};
	Measuredinfomation MeasuredThisTime;
	void InitializeEachSettings(int Numberofvehicle, int Numberoflane, int lanelength, int Cooperator);
	Constants constants;
	Car_information car;
	Map_information map;
	Random *random;
	bool isDistributed = false;
	bool isCorrelated = false;
	std::vector<int> Defectoreachlane;
	int Defector = 0;
	int Cooperator = 0;
};