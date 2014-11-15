#ifndef __OPTIMIZE_H__
#define __OPTIMIZE_H__

#include <vector>
using namespace std;

typedef double obj_fun(vector<double> &args);

class lim {
public:
	double up;
	double down;
};

class de_opt {
public:
	de_opt(int Np, int D, int g_max);
	void opt_init();
public:
	/* size of the population */
	int Np;

	/* dimension */
	int D;

	/* generation count and limit */
	int g;
	int g_max;

	/* limit of each dimension */
	vector<lim> lim_a, limit_b;
	
	/* populations */
	vector<vector<double> > vec1;
	vector<vector<double> > vec2;

	/* objective function */
	obj_fun fun;

	/* best member index and objective function value */
	int index;
	double best_value;
};

#endif
