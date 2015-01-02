#ifndef _SYSTEM_STATE_H_
#define _SYSTEM_STATE_H_

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "differential_evolution.hpp"
#include "axis.h"
#include "geometric.h"
#include "load_seam.h"
#include "job.h"
#include "teach_point.h"
#include "system_state.h"

class system_state
{
public:
	const int m_axis_nr;
	const int m_auxiliary_variable_nr;
//	int m_sub_cri_nr;

	const Vector3D& m_p;
	const Vector3D& m_n;
	const Vector3D& m_t;

	const std::vector<axis>& m_axes;
	const std::vector<axis>& m_auxiliary_variable;
	const std::vector<int>& m_map;

	const std::vector<teach_point>& m_teach_points;
	const std::vector<double>& m_weight;

public:
	std::vector<double> m_axes_values;
	std::vector<double> m_auxiliary_variable_values;

	std::vector<double> m_sub_cri_axis;
	std::vector<double> m_sub_cri_aux;
	std::vector<double> m_sub_cri_teach;

	double m_cri;
public:
	system_state(int axis_nr, 
		     int auxiliary_variable_nr,
		     const Vector3D& p,
		     const Vector3D& n,
		     const Vector3D& t,
		     const std::vector<axis>& axes,
		     const std::vector<axis>& auxiliary_variable,
		     const std::vector<int>& map,
		     const std::vector<teach_point>& teach_points,
		     const std::vector<double>& weight)
		: m_axis_nr(axis_nr), m_auxiliary_variable_nr(auxiliary_variable_nr),
		m_p(p), m_n(n), m_t(t), m_axes(axes), m_auxiliary_variable(auxiliary_variable),
		m_map(map), m_teach_points(teach_points), m_weight(weight) {
		m_axes_values.resize(m_axis_nr);
		m_auxiliary_variable_values.resize(m_auxiliary_variable_nr);
		m_sub_cri_axis.resize(m_axis_nr);
		m_sub_cri_aux.resize(m_auxiliary_variable_nr);
		m_sub_cri_teach.resize(m_teach_points.size());
	}

	/* void set_job(const Vector3D& p, const Vector3D& n, const Vector3D& t) { */
	/* 	m_p = p; */
	/* 	m_n = n; */
	/* 	m_t = t; */
	/* } */

	std::pair<double, double> get_range(int i) {
		assert(i < m_map.size());
		i = m_map[i];
		assert(m_axes.size() > 0);
		if (i < m_axes.size()) {
			return m_axes[i].get_range();
		} else {
			int tmp = i - m_axes.size();
			return m_auxiliary_variable[tmp].get_range();
		}
	}

	const system_state& operator=(const system_state& rhs) {
		m_axes_values = rhs.m_axes_values;
		m_auxiliary_variable_values = rhs.m_auxiliary_variable_values;
		m_sub_cri_axis = rhs.m_sub_cri_axis;
		m_sub_cri_aux = rhs.m_sub_cri_aux;
		m_sub_cri_teach = rhs.m_sub_cri_teach;
		m_cri = rhs.m_cri;
	}
	double get_var_value(int i) {
		assert(i < m_map.size());
		i = m_map[i];
		assert(m_axes.size() > 0);
		if (i < m_axes.size()) {
			return m_axes[i].last();
		} else {
			int tmp = i - m_axes.size();
			return m_auxiliary_variable[tmp].last();
		}
	}

	void set_vars(int i, double value) {
		assert(i < m_map.size());
		i = m_map[i];
		if (i < m_axes_values.size()) {
			m_axes_values[i] = value;
		} else {
			int tmp = i - m_axes.size();
			m_auxiliary_variable_values[tmp] = value;
		}
	}

	std::string to_string() const
	{
		std::ostringstream os;
//		os.precision(16);
//		os << std::setw(10);
		os.width(10);

		//os << "cost: " << cost() << ", vars: ";
		os << m_cri << " ";

		for (int i = 0; i < m_axes_values.size(); i++)
		{
			os.width(10);
			os << m_axes_values[i] << " ";
		}

		for (int i = 0; i < m_auxiliary_variable_values.size(); i++)
		{
			os.width(10);
			os << m_auxiliary_variable_values[i] << " ";
		}

		return os.str();
	}

	double dist(system_state& rhs) {
		double dist = 0.0;
		for (int i = 0; i < m_axes_values.size(); i++) {
			double tmp = m_axes_values[i] - rhs.m_axes_values[i];
			dist += tmp * tmp;
		}

		for (int i = 0; i < m_auxiliary_variable_values.size(); i++) {
			double tmp = m_auxiliary_variable_values[i] - rhs.m_auxiliary_variable_values[i];
			dist += tmp * tmp;
		}

		return sqrt(dist);
	}

        /* double operator() (de::DVectorPtr args) { */
	/* 	calc_criteria(); */
	/* 	return m_cri; */
	/* } */

	double criteria() {
		m_cri = 1.0;

		for (int i = 0; i < m_axes.size(); i++) {
			m_sub_cri_axis[i] = m_axes[i](m_axes_values[i]);
			m_cri *= m_sub_cri_axis[i];
		}

		for (int i = 0; i < m_auxiliary_variable.size(); i++) {
			m_sub_cri_aux[i] = m_auxiliary_variable[i](m_auxiliary_variable_values[i]);
			m_cri *= m_sub_cri_aux[i];
		}

		for (int i = 0; i < m_teach_points.size(); i++) {
			m_sub_cri_teach[i] = m_teach_points[i](m_axes_values, m_auxiliary_variable_values);
			m_cri *= m_sub_cri_teach[i];
		}		
		
		return m_cri;
	}
};

#endif /* _SYSTEM_STATE_H_ */
