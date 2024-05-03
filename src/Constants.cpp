#include "Constants.h"

void Constants::setConstants(int lanelength, int Numberoflane, int Numberofvehicle) {
	this->N = Numberofvehicle;
	this->lanelength = lanelength;
	this->Numberoflane = Numberoflane;
	this->G = 15;
	this->r = 0.99;
	this->S = 2;
	this->q = 0.99;
	this->p1 = 0.999;
	this->p2 = 0.99;
	this->p3 = 0.98;
	this->p4 = 0.01;
}