#ifndef _EXP_CRITERIA_H_
#define _EXP_CRITERIA_H_

#include <cassert>
#include "criteria.h"

class exp_criteria: public cri {
private:
	double m_mu;
	double m_sigma;
	double m_offset;
public:
exp_criteria(double mu, double sigma, double offset)
	: m_mu(mu), m_sigma(sigma), m_offset(offset) {
		assert(sigma > 0);
	}

	void set_para(double para);
	double operator() (double x);
};

#endif /* _EXP_CRITERIA_H_ */
