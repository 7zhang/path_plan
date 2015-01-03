#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <map>
#include "stdio.h"
#include "differential_evolution.hpp"
#include "axis.h"
#include "geometric.h"
#include "load_seam.h"
#include "job.h"
#include "teach_point.h"
#include "system_state.h"

using namespace de;

int program_jpos(vector<JAngle> &angle, vector<JAngle> &ex_angle, char *path);
int program_cpos(vector<RPY> &rpy, vector<JAngle> &ex_angle, char *path);


class robot_listener : public de::listener
{
public:
	virtual void start()
		{
		}

	virtual void end()
		{
		}

	virtual void error()
		{
		}

	virtual void startGeneration( size_t genCount )
		{
		}

	virtual void endGeneration( size_t genCount, individual_ptr bestIndGen, individual_ptr bestInd)
		{
			std::cout << ( boost::format( "%1%, %2%\n" ) % genCount % bestInd->to_string()).str();
		}

	virtual void startSelection( size_t genCount )
		{
		}

	virtual void endSelection( size_t genCount )
		{
		}

	virtual void startProcessors( size_t genCount )
		{
		}

	virtual void endProcessors( size_t genCount )
		{
		}
};

template <typename T>
class robot_system
{
private:
//      lock m_continue and m_i	
	boost::mutex m_mutex;
	boost::condition_variable m_cond;
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

	std::vector<T> m_states;
	std::vector<std::string> m_stl_path;
//	std::string m_seam;
	job m_job;

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

public:
robot_system(int job_id, int pop_size, int time_interval, 
	     std::vector<std::string> stl_path, std::string seam) :
	m_job_id(job_id), m_pop_size(pop_size), m_time_interval(time_interval),
		m_stl_path(stl_path), m_job(seam), m_i(-1), m_continue(1) {
		T::init(m_sys_name, m_redundancy, m_axis_nr, m_auxiliary_variable_nr, m_axes, m_auxiliary_variable, m_map, m_teach_points, m_teach_weight);
		optimize_init();
	}

robot_system(int job_id, int pop_size, int time_interval, 
	     std::vector<std::string> stl_path, job j) :
	m_job_id(job_id), m_pop_size(pop_size), m_time_interval(time_interval),
		m_stl_path(stl_path) , m_job(j), m_i(-1), m_continue(1){
		T::init(m_sys_name, m_redundancy, m_axis_nr, m_auxiliary_variable_nr, m_axes, m_auxiliary_variable, m_map, m_teach_points, m_teach_weight);
		optimize_init();
	}

	void optimize_init() {
		m_thread_nr = 4;
		m_weight = 0.4;
		m_crossover = 0.9;
	}
	void operator()();
	void set_de_args(int pop_size, int thread_nr, double weight, double crossover);

	int push_value(T& s) {
		for (int i = 0; i < s.m_axes_values.size(); i++) {
			if (m_axes[i].add_value(s.m_axes_values[i])) {
				return -1;
			}
		}

		for (int i = 0; i < s.m_auxiliary_variable.size(); i++) {
			if (m_auxiliary_variable[i].add_value(s.m_auxiliary_variable_values[i])) {
				return -1;
			}
		}

		return 0;
	}

	std::string get_sys_info() {
		std::stringstream ss;
		ss << "system: " << m_sys_name << std::endl
		   << "redundancy: " << m_redundancy << std::endl;

		return ss.str();
	}

	std::pair<int, int> get_finish_rate() {
		boost::unique_lock<boost::mutex> lock(m_mutex);
//		cout << "m_continue: " << m_continue << endl;
		if (m_continue != 0) {
			m_cond.wait(lock);
		}
		int size = m_job.get_size();

		if (m_continue == 0 && m_i < size - 1) {
			return make_pair(-size, m_i + 1);
		} else {
			return make_pair(size, m_i + 1);
		}
	}

	int set_sys_parameter(std::string para_name, void *para_value) {
		boost::unique_lock<boost::mutex> lock(m_mutex);
		if (m_continue != 0) {
			m_continue = 0;
		}

		int tmp_i;
		double tmp_d;
		switch (para_name) {
		case "de_pop_size":
			tmp_i = *(int *)para_value;
			if (tmp_i < 0) {
				std::cerr << "can't set de_pop_size to " << tmp_i
					  << ", value illegal" << std::endl;
				return -1;
			}
			m_pop_size = tmp_i;
			std::cerr << "de_pop_size in job " << m_job_id
				  << "changed to " << m_pop_size << std::endl;
			break;
		case "de_thread_nr":
			tmp_i = *(int *)para_value;
			if (tmp_i < 0) {
				std::cerr << "can't set de_thread_nr to " << tmp_i
					  << ", value illegal" << std::endl;
				return -1;
			}
			m_thread_nr = tmp_i;
			std::cerr << "de_thread_nr in job " << m_job_id
				  << "changed to " << m_thread_nr << std::endl;
			break;
		case "de_weight":
			tmp_d = *(double *)para_value;
			if (tmp_d < 0.0 || tmp_d > 2.0) {
				std::cerr << "can't set de_weight to " << tmp_d
					  << ", value illegal" << std::endl;
				return -1;
			}

			m_weight = tmp_d;
			std::cerr << "de_weight in job " << m_job_id
				  << "changed to " << m_weight << std::endl;
			break;
		case "de_crossover":
			tmp_d = *(double *)para_value;
			if (tmp_d < 0.0 || tmp_d > 1.0) {
				std::cerr << "can't set de_crossover to " << tmp_d
					  << ", value illegal" << std::endl;
				return -1;
			}

			m_crossover = tmp_d;
			std::cerr << "de_crossover in job " << m_job_id
				  << "changed to " << m_crossover << std::endl;
			break;
		}

		return 0;
	}
};

template <typename T>
void robot_system<T>::operator()()
{
//	T::init();
	T pre_state(m_axis_nr,m_auxiliary_variable_nr, m_job.get_p(0), m_job.get_n(0), m_job.get_t(0),
		    m_axes, m_auxiliary_variable, m_map, m_teach_points, m_teach_weight);
	pre_state.m_axes_values[0] = 15.0;
	pre_state.m_axes_values[1] = -95.0/2;
	pre_state.m_axes_values[2] = 15.0;
	pre_state.m_axes_values[3] = 0.0;
	pre_state.m_axes_values[4] = 0.0;
	pre_state.m_axes_values[5] = 0.0;
	pre_state.m_axes_values[6] = 45.0;
	pre_state.m_axes_values[7] = 0.0;
	pre_state.m_axes_values[8] = -50.0;

	std::vector<JAngle> best_angle;
	std::vector<JAngle> best_ex_angle;

	int err_count = 0;

	for (int i = 0; i < m_job.get_size(); i++) {
//	for (int i = 0; i < 1; i++) {
//		cerr << "job " << m_job_id << ": ";
//		cerr << "i = " << i << ", ";
		T cur_state(m_axis_nr,m_auxiliary_variable_nr, m_job.get_p(i), m_job.get_n(i), m_job.get_t(i),
		    m_axes, m_auxiliary_variable, m_map, m_teach_points, m_teach_weight);
//		cur_state.set_job(m_job.get_p(i), m_job.get_n(i), m_job.get_t(i));
			
		de::constraints_ptr constraints( boost::make_shared< de::constraints >(m_redundancy, -1.0e6, 1.0e6));
		for (int j = 0; j < m_redundancy; j++) {
			std::pair<double, double> tmp = cur_state.get_range(j);
//			std::cout << "j = " << j << ":" << tmp.first << ", " << tmp.second << endl;
			(*constraints)[j] = boost::make_shared<de::real_constraint>(tmp.first, tmp.second);
//			double pre = pre_state.get_var_value(j);
//			(*constraints)[j] = boost::make_shared<de::real_constraint>(pre_state.m_axes_values, tmp.second);
		}

		de::listener_ptr listener( boost::make_shared< de::null_listener >() );
//		de::listener_ptr listener( boost::make_shared< robot_listener >() );
		de::processor_listener_ptr processor_listener( 
			boost::make_shared< de::null_processor_listener >() );
		typename de::processors< T >::processors_ptr _processors( 
			boost::make_shared< de::processors< T > >( 
				m_thread_nr, boost::ref( cur_state ), processor_listener ) );
		/* de::termination_strategy_ptr terminationStrategy(  */
		/* 	boost::make_shared< de::max_gen_termination_strategy >( 300 ) ); */

		de::termination_strategy_ptr terminationStrategy( 
			boost::make_shared< de::min_devitaion_termination_strategy >( 1e-10 ) );

		de::selection_strategy_ptr selectionStrategy(
			boost::make_shared< de::tournament_selection_strategy >() );
		de::mutation_strategy_arguments mutation_arguments( m_weight, m_crossover );
		de::mutation_strategy_ptr mutationStrategy(
			boost::make_shared< de::mutation_strategy_1 >( m_redundancy, mutation_arguments ) );
		typename de::differential_evolution< T > de(
			m_redundancy, m_pop_size, _processors, constraints, false, 
			terminationStrategy, selectionStrategy, mutationStrategy, listener );

		de.run();
		de::individual_ptr best = de.best();

		std::cerr << best->cost() << endl;

		double cost = cur_state(best->vars());
		double diff = cur_state.dist(pre_state);

		if (i > 0 && diff > 200 && err_count < 100) {
			err_count++;
			i--;
			continue;
		}

		if (err_count > 99) {
			std::cerr << "failled but tried" << std::endl;
		}
		err_count = 0;

//		std::cout << cur_state.to_string() << std::endl;

//		cur_state.check();
//		std::cout << "cost: ";
//		std::cout << cost;
//		std::cout << std::endl;
//		std::cout << "axes: ";
		/* for (int j = 0; j < cur_state.m_axes_values.size(); j++) { */
		/* 	std::cout << " " << cur_state.m_axes_values[j]; */
		/* } */
		
//		std::cout << std::endl << "auxiliary: ";
		/* for (int j = 0; j < cur_state.m_auxiliary_variable_values.size(); j++) { */
		/* 	std::cout << " " << cur_state.m_auxiliary_variable_values[j]; */
		/* } */

//		std::cout << std::endl;
		if (push_value(cur_state)) {
			std::cout << "state illegal" << std::endl;
			break;
		}

		JAngle besta(cur_state.m_axes_values[0], cur_state.m_axes_values[1], cur_state.m_axes_values[2],
			     cur_state.m_axes_values[3], cur_state.m_axes_values[4], cur_state.m_axes_values[5]);
		JAngle bestea(cur_state.m_axes_values[6], cur_state.m_axes_values[7], cur_state.m_axes_values[8], 
			      0.0, 0.0, 0.0);

		best_angle.push_back(besta);
		best_ex_angle.push_back(bestea);

		/* i--; */
		/* continue; */

		pre_state = cur_state;
//		std::cout << std::endl;

		boost::unique_lock<boost::mutex> lock(m_mutex);
		m_cond.notify_one();
		if (! m_continue) {
			break;
		}
		m_i = i;
	}

	boost::unique_lock<boost::mutex> lock(m_mutex);
	m_continue = 0;
	m_cond.notify_one();

	program_jpos(best_angle, best_ex_angle, "./program.glp");
}

template <typename T>
void robot_system<T>::set_de_args(int pop_size, int thread_nr, double weight, double crossover) {
	assert(pop_size > 0 && thread_nr > 0 && weight > 0.0 && crossover > 0.0);
	m_pop_size = pop_size;
	m_thread_nr = thread_nr;
	m_weight = weight;
	m_crossover = crossover;
}

int program_jpos(vector<JAngle> &angle, vector<JAngle> &ex_angle, char *path)
{
	// for (int i = 1; i < angle.size(); i++) {
	// 	to_continuous(angle[i], angle[i - 1]);
	// }
	FILE *file;
	if((file = fopen(path, "wb")) == NULL) {
		printf("open file %s error\n", path);
		return -1;
	}
	
	fwrite("//DATASEG\n", 1, 10, file);
	for (int i = 0; i < angle.size(); i++) {
		fprintf(file, "JPOS: loc%d=(%f,%f,%f,%f,%f,%f,%f,%f,%f)\n",
			i + 1, angle[i].get_angle(1), angle[i].get_angle(2), angle[i].get_angle(3),
			angle[i].get_angle(4), angle[i].get_angle(5), angle[i].get_angle(6),
			ex_angle[i].get_angle(1), ex_angle[i].get_angle(2), ex_angle[i].get_angle(3));
	}
	
	fwrite("//PROGRAMSEG\n", 1, 13, file);
	
	for (int j = 0; j < angle.size(); j++) {
		fprintf(file, "MOVJ (loc%d,Vel=5.000,Acc=100.000,Jerk=100.000)\n", j + 1);
	}
	
	return 0;
}

int program_cpos(vector<RPY> &rpy, vector<JAngle> &ex_angle, char *path)
{
	FILE *file;
	if((file = fopen(path, "wb")) == NULL) {
		printf("open file %s error\n", path);
		return -1;
	}
	
	fwrite("//DATASEG\n", 1, 10, file);
	for (int i = 0; i < rpy.size(); i++) {
		fprintf(file, "CPOS: loc%d=(%f,%f,%f,%f,%f,%f,%f,%f,%f)\n",
			i + 1, rpy[i].pos.dx, rpy[i].pos.dy, rpy[i].pos.dz,
			rpy[i].orient.dx, rpy[i].orient.dy, rpy[i].orient.dz,			
			ex_angle[i].get_angle(1), ex_angle[i].get_angle(2), ex_angle[i].get_angle(3));
	}
	
	fwrite("//PROGRAMSEG\n", 1, 13, file);
	
	for (int j = 0; j < rpy.size(); j++) {
		fprintf(file, "MOVL (loc%d,Vel=5.000,Acc=100.000,Jerk=100.000)\n", j + 1);
	}
	
	return 0;
}
#endif /* _ROBOT_H_ */
