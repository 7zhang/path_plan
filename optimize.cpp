#include <random>
#include "optimize.h"

de_opt::de_opt(int Np, int D, int g_max,
	       vector<lim> &lim_a,
	       vector<lim> &lim_b,
	       obj_fun fun)
{
	this->Np = Np;
	this->D = D;
	int g = 0;
	this->g_max = g_max;
	this->lim_a = lim_a;
	this->lim_b = lim_b;
	vec1.resize(Np);
	vec2.resize(Np);
	for (int i = 0; i < Np; i++) {
		vec1[i].resize(D);
		vec2[i].resize(D);
	}

	this->fun = fun;

	index = -1;
	best_value = 0;
	opt_init();
}

void de_opt::opt_init()
{
	std::random_device rd;
	std::mt19937 gen(rd());
}
