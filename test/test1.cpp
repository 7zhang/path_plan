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

int main(int argc, char *argv[])
{
	std::vector<double> para;
	KR5ARC_robot::init(m_sys_name, m_redundancy, m_axis_nr, m_auxiliary_variable_nr, m_axes, m_auxiliary_variable, m_map, m_teach_points, m_teach_weight, para);
	KR5ARC_robot cur_state(m_axis_nr,m_auxiliary_variable_nr, 0, para, m_job.get_p(i), m_job.get_n(i), m_job.get_t(i),
		    m_axes, m_auxiliary_variable, m_map, m_teach_points, m_teach_weight);
}
