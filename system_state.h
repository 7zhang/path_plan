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
	static int m_axis_nr;
	static int m_auxiliary_variable_nr;
//	static int m_sub_cri_nr;

	static Vector3D m_p;
	static Vector3D m_n;
	static Vector3D m_t;

	static std::vector<axis> m_axes;
	static std::vector<axis> m_auxiliary_variable;
	static std::vector<int> m_map;

	static std::vector<teach_point> m_teach_points;
	static std::vector<double> m_weight;

public:
	std::vector<double> m_axes_values;
	std::vector<double> m_auxiliary_variable_values;

	std::vector<double> m_sub_cri_axis;
	std::vector<double> m_sub_cri_aux;
	std::vector<double> m_sub_cri_teach;

	double m_cri;
public:
	system_state() {
		m_axes_values.resize(m_axis_nr);
		m_auxiliary_variable_values.resize(m_auxiliary_variable_nr);
		m_sub_cri_axis.resize(m_axis_nr);
		m_sub_cri_aux.resize(m_auxiliary_variable_nr);
		m_sub_cri_teach.resize(m_teach_points.size());
	}

	void set_job(const Vector3D& p, const Vector3D& n, const Vector3D& t) {
		m_p = p;
		m_n = n;
		m_t = t;
	}

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

	int push_value() {
		for (int i = 0; i < m_axes_values.size(); i++) {
			if (m_axes[i].add_value(m_axes_values[i])) {
				return -1;
			}
		}

		for (int i = 0; i < m_auxiliary_variable.size(); i++) {
			if (m_auxiliary_variable[i].add_value(m_auxiliary_variable_values[i])) {
				return -1;
			}
		}

		return 0;
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
