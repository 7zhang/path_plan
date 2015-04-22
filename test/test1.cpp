#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <string>
#include <cmath>
#include "robot.h"
#include "kuka/kr5arc_robot.h"
int m_continue;
int m_i;

int m_job_id;
std::string m_sys_name;
int m_redundancy;
int m_time_interval;

int m_pop_size;
int m_thread_nr;
double m_weight;
double m_crossover;

double recommend;
std::vector<std::string> m_stl_path;
//	std::string m_seam;

int m_axis_nr;
int m_auxiliary_variable_nr;
//	static int m_sub_cri_nr;

/* Vector3D m_p; */
/* Vector3D m_n; */
/* Vector3D m_t; */

std::vector<axis> m_axes;
std::vector<axis> m_auxiliary_variable;
std::vector<int> m_map;

std::vector<teach_point> m_teach_points;
std::vector<double> m_teach_weight;

string stl_load_path;
KR5ARC_robot *my_rob;
de::DVectorPtr p_var;

double cc(double var)
{
	(*p_var)[3] = var;
	return (*my_rob)(p_var);
}

int main(int argc, char *argv[])
{
	stl_load_path = "intersect_stl";
	std::vector<double> para;
	para.push_back(0);
	para.push_back(0);
	std::vector<std::string> stl_path;
	Vector3D m_p(0, 0, 0), m_n(1, 0, 0), m_t(0, 1, 0);
	std::vector<Vector3D> tmpp, tmpn, tmpt;
	
	tmpp.push_back(m_p);
	tmpn.push_back(m_n);
	tmpt.push_back(m_t);
	job start_point(0, para, tmpp, tmpn, tmpt);;

	KR5ARC_robot::init(m_sys_name, m_redundancy, m_axis_nr, m_auxiliary_variable_nr, m_axes, m_auxiliary_variable, m_map, m_teach_points, m_teach_weight, para);
	my_rob = new KR5ARC_robot(m_axis_nr,m_auxiliary_variable_nr, 0, para, start_point.get_p(0), start_point.get_n(0), start_point.get_t(0),
		    m_axes, m_auxiliary_variable, m_map, m_teach_points, m_teach_weight);
	de::DVector var;
	var.push_back(0);
	var.push_back(0);
	var.push_back(0);
	var.push_back(0);
	var.push_back(0);
	var.push_back(0);
	var.push_back(0);
	p_var = boost::make_shared< de::DVector >(var);
	double tmp = cc(3);
}
