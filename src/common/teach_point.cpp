#include "teach_point.h"
#include "axis.h"
#include "exp_criteria.h"
#include <vector>
#include <cmath>

double teach_point::operator() (const std::vector<axis>& axes, const std::vector<axis>& auxiliary_variable,
		   const std::vector<double>& axes_values, const std::vector<double>& auxiliary_variable_values, 
		   const Vector3D& position) const {
	
	double cri = 1.0;
	Vector3D diff = m_position - position;
	double dist = diff.get_length();
	exp_criteria ratio_c(0.0, m_effect_sigma, 0.0);
	double ratio = ratio_c(dist);

	for (int i = 0; i < m_mu_axis.size(); i++) {
		double tmp = axes_values[i] - m_mu_axis[i];
		double variable_sigma = axes[i].length() * 0.05;
		cri *= exp(- tmp * tmp / 2 / variable_sigma / variable_sigma * ratio);
	}

	for (int i = 0; i < m_mu_aux.size(); i++) {
		double tmp = auxiliary_variable_values[i] - m_mu_aux[i];
		double variable_sigma = auxiliary_variable[i].length() * 0.05;
		cri *= exp(- tmp * tmp / 2 / variable_sigma / variable_sigma * ratio);
	}

	return cri;
}
