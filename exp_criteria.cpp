#include "exp_criteria.h"
#include <cmath>

double exp_criteria::operator() (double x)
{
	if (m_sigma == 0.0) {
		return x;
	}
	double tmp = x - m_mu;
	return exp(- tmp * tmp / 2 / m_sigma / m_sigma) + m_offset;
}

void exp_criteria::set_para(double para)
{
	m_offset = para;
}
