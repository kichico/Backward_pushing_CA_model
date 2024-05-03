#include "Multilane_TrafficFlow.h"

int main(/*int argc, char* argv[]*/) {
//	if (argc == 1) return 0;
	int filenumber = 1;// std::stoi(argv[1]);
	Multilane_TrafficFlow mt;
	int NumberofLane = 3;
	int Length = 1000;
	int N = NumberofLane * Length;
	mt.Calculation(filenumber,Length, N, NumberofLane);
	return 0;
}
