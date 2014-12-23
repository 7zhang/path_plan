#include "teach_point.h"
#include "exp_criteria.h"

double teach_point::operator() (std::vector<double> &arg_axis, std::vector<double> &arg_aux) {
	double cri = 1.0;
	double sum = 0.0;
	for (int i = 0; i < m_mu_axis.size(); i++) {
		exp_criteria tmp(m_mu_axis[i], m_sigma_axis[i], m_offset_axis[i]);
		cri += tmp(arg_axis[i]) * m_weight_axis[i];
		sum += m_weight_axis[i];
	}

	// for (int i = 0; i < m_aux.size(); i++) {
	// 	exp_criteria tmp(m_mu_aux[i], m_sigma_aux[i], m_offset_aux[i]);
	// 	cri += tmp(arg_aux[i]) * m_weight_aux[i];
	// 	sum += m_weight_aux[i];
	// }

	return cri / sum;
}
