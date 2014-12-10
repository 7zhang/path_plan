#ifndef _TEACH_POINT_H_
#define _TEACH_POINT_H_

#include <vector>

class teach_point
{
private:
	std::vector<double> m_axis;
	std::vector<double> m_mu_axis;
	std::vector<double> m_sigma_axis;
	std::vector<double> m_offset_axis;
	std::vector<double> m_weight_axis;

	std::vector<double> m_aux;
	std::vector<double> m_mu_aux;
	std::vector<double> m_sigma_aux;
	std::vector<double> m_offset_aux;
	std::vector<double> m_weight_aux;
public:
	teach_point(std::vector<double>& axis, std::vector<double>& mu_axis,
		    std::vector<double>& sigma_axis, std::vector<double>& offset_axis,
		    std::vector<double>& weight_axis, std::vector<double>& aux,
		    std::vector<double>& mu_aux, std::vector<double>& sigma_aux,
		    std::vector<double>& offset_aux, std::vector<double>& weight_aux)
		: m_axis(axis), m_mu_axis(mu_axis), m_sigma_axis(sigma_axis), 
		  m_offset_axis(offset_axis), m_weight_axis(weight_axis),
		  m_aux(aux), m_mu_aux(mu_aux), m_sigma_aux(sigma_aux), 
		  m_offset_aux(offset_aux), m_weight_aux(weight_aux) {}
	double operator() (std::vector<double> &arg_axis, std::vector<double> &arg_aux);
	virtual ~teach_point();
};

#endif /* _TEACH_POINT_H_ */
