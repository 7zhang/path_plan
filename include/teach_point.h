#ifndef _TEACH_POINT_H_
#define _TEACH_POINT_H_

#include <vector>
#include "geometric.h"
#include "axis.h"

class teach_point
{
private:
// teach point coordinates
	Vector3D m_position;
// effect range
	double m_effect_sigma;
	std::vector<double> m_mu_axis;
	std::vector<double> m_mu_aux;
public:
teach_point(const Vector3D& position, double effect_sigma, std::vector<double>& mu_axis, std::vector<double>& mu_aux)
	: m_position(position), m_effect_sigma(effect_sigma), m_mu_axis(mu_axis), m_mu_aux(mu_aux) {}

	double operator() (const std::vector<axis>& axes, const std::vector<axis>& auxiliary_variable,
			   const std::vector<double>& axes_values, const std::vector<double>& auxiliary_variable_values, 
			   const Vector3D& position) const;

};

#endif /* _TEACH_POINT_H_ */
