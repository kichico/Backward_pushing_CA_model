#pragma once
#include <vector>

//This class store the information of vechiles based on ID
class Car_information {
private:
	struct CurrentandPreviousvector {
		std::vector<int> current;
		std::vector<int> previous;
	};
	struct PrecedingandFollowingcarID {
		CurrentandPreviousvector preceding;
		CurrentandPreviousvector following;
	};
	struct PushingChecker {
		std::vector<int> Preceding;
		std::vector<bool> isPushing;
	};
public:
	CurrentandPreviousvector position, velocity, headway,lanenumber;
	PrecedingandFollowingcarID around;
	PushingChecker pushing;
	struct SignalKind {
		static const int Non = 0;
		static const int Right = 1;
		static const int Left = 2;
	};
	struct StrategyKind {
		static const int C = 0;
		static const int D = 1;
	};
	struct LeadingVehicle {
		int ID = 0;
		int distance = 0;
		bool existence = 0;
	};
	std::vector<LeadingVehicle> leadingvehicle;
	std::vector<int> signal;
	std::vector<int> strategy;
	std::vector<int> canditate_velocity;
	std::vector<int> S;
	std::vector<int> Vmax;
	std::vector<int> List_Defector;
	std::vector<int> List_Cooperator;
	void Initialize(int Numberofvehicle);
	void Fromcurrent_toprevious();

};
