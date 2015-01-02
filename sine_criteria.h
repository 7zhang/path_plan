#ifndef _SINE_CRITERIA_H_
#define _SINE_CRITERIA_H_

#include <cassert>
#include <boost/math/constants/constants.hpp>
#include "criteria.h"

class sine_criteria: public cri {
private:
	double m_min;
	double m_max;
	double m_lamda;
	double m_pi;
public:
sine_criteria(double min, double max, double lamda)
	: m_min(min), m_max(max), m_lamda(lamda) {
		assert(min < max);
		assert(lamda > 0);
		m_pi = boost::math::constants::pi<double>();
	}

	void set_para(double para);
	
	double operator() (double x) const;
};

#endif /* _SINE_CRITERIA_H_ */
