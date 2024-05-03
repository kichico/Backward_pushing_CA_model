#pragma once
#include "Lane_Change.h"

class Update_Position : public Lane_Change {
public:
	Update_Position(int filenumber, int lanelength, int Numberofvehicle, int Numberoflane, int Cooperator);
	~Update_Position();
	Measuredinfomation measured;
	bool DoMeasure = false;
	void Update_EachVehiclePosition();
	Measuredinfomation AddThisTimeResult();
	struct AverageVelocity {
		int Defector;
		int Cooperator;
		AverageVelocity operator+=(const AverageVelocity& other) {
			this->Defector += other.Defector;
			this->Cooperator += other.Cooperator;
			return *this;
		}
		void init() {
			Defector = 0;
			Cooperator = 0;
		}
	};
	AverageVelocity AverageVelocityThisTime;
	struct LaneOccupationInfo {
		struct Agent {
			int C;
			int D;
			Agent() {
				this->C = 0;
				this->D = 0;
			}
		};
		std::vector<Agent> Counter;
	};
	LaneOccupationInfo Occupation;
	void calculateOccupationRate();
private:
	int lanevelocity = 0;
	int MeasuringPoint = 100;
	bool _UpdateEachLane(int leaderID);
	Measuredinfomation _Measure(Measuredinfomation& MeasuredThistime);
};