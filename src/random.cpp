#include "random.h"

int Random::create_int_rand(std::mt19937 mt, int xmin, int xmax) {
	std::uniform_int_distribution<> rd(xmin, xmax);
	return rd(mt);
}

double Random::create_double_rand(std::mt19937 mt, double xmin, double xmax) {
	std::uniform_real_distribution<> rd(xmin, xmax);
	return rd(mt);
}

int Random::random(int N) {
	std::random_device rnd;
	std::mt19937 mt(rnd());
	return create_int_rand(mt, 0, N);
}

int Random::random(int Nmin, int Nmax) {
	std::random_device rnd;
	std::mt19937 mt(rnd());
	return create_int_rand(mt, Nmin, Nmax);
}

double Random::random(double D) {
	std::random_device rnd;
	std::mt19937 mt(rnd());
	return create_double_rand(mt, 0.0, D);
}

double Random::random(double Dmin, double Dmax) {
	std::random_device rnd;
	std::mt19937 mt(rnd());
	return create_double_rand(mt, Dmin, Dmax);
}