#ifndef _SYSTEM_STATE_H_
#define _SYSTEM_STATE_H_

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "de/differential_evolution.hpp"
#include "axis.h"
#include "geometric.h"
#include "robotdata.h"
#include "robotkinematic.h"
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


	double angle_between(Vector3D& lhs, Vector3D& rhs)
	{
		double llen = lhs.get_length();
		double rlen = rhs.get_length();
		double dot = lhs ^ rhs;

		assert(llen > 1e-6 && rlen > 1e-6);
		double tmp = dot / llen / rlen;
		// if(!(tmp <= 1.0 && tmp >= -1.0)) {
		// 	std::cout << "tmp = " << tmp << std::endl;
		// 	std::cout << "llen = " << llen << std::endl;
		// 	std::cout << "rlen = " << rlen << std::endl;
		// 	std::cout << "dot = " << dot << std::endl;
		// }
		double pi = boost::math::constants::pi<double>();
		assert(tmp <= 1.0 && tmp >= -1.0);
//	return tmp;
		return acos(tmp) / pi * 180.0;
	}
	virtual TRANS get_gun_in_seam(const JAngle& weld_angle) = 0;
	virtual TRANS getTransWorldToWorkpiece(JAngle ex_angle) = 0;
	virtual TRANS getTrans6ToTorch() = 0;
	virtual TRANS getTransWorldToBase(JAngle ex_angle) = 0;
	virtual bool InverseRobot(JAngle& Jointangle,const JAngle& lastJointangle,const TRANS& t6) =0;
	virtual double get_jacobi_deter(JAngle& angle) = 0;

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

	system_state& operator=(const system_state& rhs) {
		m_axes_values = rhs.m_axes_values;
		m_auxiliary_variable_values = rhs.m_auxiliary_variable_values;
		m_sub_cri_axis = rhs.m_sub_cri_axis;
		m_sub_cri_aux = rhs.m_sub_cri_aux;
		m_sub_cri_teach = rhs.m_sub_cri_teach;
		m_cri = rhs.m_cri;
	}

	/* system_state(const system_state& rhs): m_axis_nr(rhs.m_axis_nr), m_auxiliary_variable_nr(rhs.m_auxiliary_variable_nr), m_p(rhs.m_p), m_n(rhs.m_n), m_t(rhs.m_t), m_axes(rhs.m_axes), m_auxiliary_variable(rhs.m_auxiliary_variable), m_map(rhs.m_map), m_teach_points(rhs.m_teach_points), m_weight(rhs.m_weight) { */
	/* 	m_axes_values = rhs.m_axes_values; */
	/* 	m_auxiliary_variable_values = rhs.m_auxiliary_variable_values; */
	/* 	m_sub_cri_axis = rhs.m_sub_cri_axis; */
	/* 	m_sub_cri_aux = rhs.m_sub_cri_aux; */
	/* 	m_sub_cri_teach = rhs.m_sub_cri_teach; */
	/* 	m_cri = rhs.m_cri; */
	/* } */

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

		for (int i = 0; i < m_sub_cri_axis.size(); i++)
		{
			os.width(10);
			os << m_sub_cri_axis[i] << " ";
		}

		for (int i = 0; i < m_sub_cri_aux.size(); i++)
		{
			os.width(10);
			os << m_sub_cri_aux[i] << " ";
		}

		for (int i = 0; i < m_sub_cri_teach.size(); i++)
		{
			os.width(10);
			os << m_sub_cri_teach[i] << " ";
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

	double operator() (de::DVectorPtr args) {
		std::cout << "this is embarrassing" << std::endl;
		for (int i = 0; i < (*args).size(); i++) {
			set_vars(i, (*args)[i]);
		}
		// state s;
		// state pre_s;

		JAngle pre_angle = JAngle(m_axes[0].last(), m_axes[1].last(), m_axes[2].last(),
					  m_axes[3].last(), m_axes[4].last(), m_axes[5].last());
//	JAngle pre_ex_angle = JAngle(m_axes[6].last(), m_axes[7].last(), m_axes[8].last(),
//				0.0, 0.0, 0.0);
		// m_s.in.ex1 = (*args)[0];
		// m_s.in.ex2 = (*args)[1];
		// m_s.in.ex3 = (*args)[2];
		// m_s.in.pusi = (*args)[3];
		// m_s.in.theta = (*args)[4];
		// m_s.in.fi = (*args)[5];

		double pi = boost::math::constants::pi<double>();
		JAngle gun_angle((*args)[3], (*args)[4], (*args)[5]);
		TRANS gun_in_seam = this->get_gun_in_seam(gun_angle);
		/* // TRANS gun_in_seam(0.0, 0.0, -1.0,  */
		/* // 		  1.0, 0.0, 0.0,  */
		/* // 		  0.0, -1.0, 0.0, */
		/* // 		  0.0, 0.0, 0.0); */
		/* TRANS rotateY(cos((*args)[3] / 180.0 * pi), 0.0, -sin((*args)[3] / 180.0 * pi), */
		/* 	      0.0, 1.0, 0.0, */
		/* 	      sin((*args)[3] / 180.0 * pi), 0.0, cos((*args)[3] / 180.0 * pi), */
		/* 	      0.0, 0.0, 0.0); */
		/* TRANS rotateXL(1.0, 0.0, 0.0, */
		/* 	       0.0, cos((*args)[4] / 180.0 * pi), sin((*args)[4] / 180.0 * pi), */
		/* 	       0.0, -sin((*args)[4] / 180.0 * pi), cos((*args)[4] / 180.0 * pi), */
		/* 	       0.0, 0.0, 0.0); */

		/* TRANS rotateXR(1.0, 0.0, 0.0, */
		/* 	       0.0, cos((*args)[5] / 180.0 * pi), sin((*args)[5] / 180.0 * pi), */
		/* 	       0.0, -sin((*args)[5] / 180.0 * pi), cos((*args)[5] / 180.0 * pi), */
		/* 	       0.0, 0.0, 0.0); */

		/* // TRANS rotateZ(cos((*args)[5] / 180.0 * pi), sin((*args)[5] / 180.0 * pi), 0.0, */
		/* // 	      -sin((*args)[5] / 180.0 * pi), cos((*args)[5] / 180.0 * pi), 0.0, */
		/* // 	      0.0, 0.0, 1.0, */
		/* // 	      0.0, 0.0, 0.0); */
		/* gun_in_seam = rotateXL * rotateY * gun_in_seam * rotateXR; */

//	print_trans("gun_in_seam", gun_in_seam);
	
		Vector3D axis_y = m_n * m_t;
		TRANS seam_in_part(m_t.dx, m_t.dy, m_t.dz,
				   axis_y.dx, axis_y.dy, axis_y.dz,
				   m_n.dx, m_n.dy, m_n.dz,
				   m_p.dx, m_p.dy, m_p.dz);
		TRANS gun_in_part = seam_in_part * gun_in_seam;

//	print_trans("gun_in_part", gun_in_part);

		JAngle ex_angle((*args)[0], (*args)[1], (*args)[2], 0.0, 0.0, 0.0);
		TRANS part_trans;
		part_trans = this->getTransWorldToWorkpiece(ex_angle);

		Vector3D n1, t1;
		Vector3D up(0.0, 0.0, 1.0);
		n1 = part_trans * m_n;
		t1 = part_trans * m_t;
		m_auxiliary_variable_values[4] = angle_between(n1, up);
		m_auxiliary_variable_values[5] = angle_between(t1, up);
				
		TRANS torch_in_world;
		torch_in_world = part_trans * gun_in_part;

//	print_trans("torch_in_world", torch_in_world);
					
		TRANS t6_to_torch;
		t6_to_torch = this->getTrans6ToTorch();
		t6_to_torch.inverse();
				
		TRANS t6_in_world;
		t6_in_world = torch_in_world * t6_to_torch;
			
		TRANS w_to_base;
		w_to_base = this->getTransWorldToBase(ex_angle);
		w_to_base.inverse();
			
		TRANS t6_in_robot;
		t6_in_robot = w_to_base * t6_in_world;

//	print_trans("t6_in_robot", t6_in_robot);
	
		JAngle angle;
		if (InverseRobot(angle, pre_angle, t6_in_robot)) {
#ifdef DEBUG
			printf("error inverserobot\n");
#endif
//		delete rob;
			return std::numeric_limits<double>::quiet_NaN();
		}

		// TRANS t01, t02, t03, t04, t05, t06;
		// Transform::getTransBaseToJoints(angle, t01, t02, t03, t04, t05, t06);

		// TRANS t6_in_robot1 = t01 * t02 * t03 * t04 * t05 * t06;

		// print_trans("t6_in_robot1", t6_in_robot1);

		// TRANS t6;
		// rob->PositiveRobot(angle, t6);

//	print_trans("t6", t6);

		// TRANS torch_in_world1 = Transform::getTransWorldToBase(ex_angle) * t6 * Transform::getTrans6ToTorch();

//	print_trans("torch_in_world1", torch_in_world1);

		for (int i = 0; i < 6; i++)
			m_axes_values[i] = angle.get_angle(i + 1);
	
		// to_continuous(s->angle, pre_s->angle);
		// to_continuous(s->ex_angle, pre_s->ex_angle);
//	delete rob;


	
		m_auxiliary_variable_values[3] = this->get_jacobi_deter(angle);

//	check();

		return criteria();
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
