#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <iostream>
#include <vector>
#include <string>

#include "differential_evolution.hpp"
#include "axis.h"
#include "geometric.h"
#include "load_seam.h"
#include "job.h"
#include "teach_point.h"
#include "system_state.h"

template <typename T>
class robot_system
{
private:
	std::string m_sys_name;
	int m_redundancy;
	int m_time_interval;

	int m_pop_size;
	int m_thread_nr;
	double m_weight;
	double m_crossover;

	std::vector<T> m_states;
	std::vector<std::string> m_stl_path;
//	std::string m_seam;
	job m_job;
public:
robot_system(std::string sys_name, int redundancy, int pop_size, int time_interval, 
	     std::string stl_path, std::string seam):
	m_sys_name(sys_name), m_redundancy(redundancy),
		m_pop_size(pop_size), m_time_interval(time_interval),
		m_stl_path(stl_path), m_job(seam) {
		m_thread_nr = 4;
		m_weight = 0.8;
		m_crossover = 0.9;
	}
	void path_plan();
	void set_de_args(int pop_size, int thread_nr, double weight, double crossover);
	
	~robot_system();
};

#endif /* _ROBOT_H_ */
