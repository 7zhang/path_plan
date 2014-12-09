#include "exp_criteria.h"
#include <cmath>

double exp_criteria::operator() (double x)
{
	double tmp = x - m_mu;
	return exp(- tmp * tmp / 2 / m_sigma / m_sigma) + m_offset;
}

void exp_criteria::set_para(double para)
{
	m_offset = para;
}
