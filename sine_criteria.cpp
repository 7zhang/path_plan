#include "sine_criteria.h"
#include <cmath>

double sine_criteria::operator() (double x)
{
	if (x <= m_min || x >= m_max) {
		return 0.0;
	} else {
		double mid = (m_min + m_max) / 2.0;
		double len = m_max - m_min;
		double cosine = cos((x - mid) / len * m_pi);
		return pow(cosine, m_lamda);
	}
}

void sine_criteria::set_para(double para)
{
	assert(para > 0);
	m_lamda = para;
}
