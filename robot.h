#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <iostream>
#include <vector>
#include <string>

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
	     std::vector<std::string> stl_path, std::string seam):
	m_sys_name(sys_name), m_redundancy(redundancy),
		m_pop_size(pop_size), m_time_interval(time_interval),
		m_stl_path(stl_path), m_job(seam) {
		m_thread_nr = 4;
		m_weight = 0.4;
		m_crossover = 0.9;
	}
	void path_plan();
	void set_de_args(int pop_size, int thread_nr, double weight, double crossover);
};


template <typename T>
void robot_system<T>::path_plan()
{
	T::init();
	T pre_state;
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
		cerr << "i = " << i << std::endl;
		T cur_state;
		cur_state.set_job(m_job.get_p(i), m_job.get_n(i), m_job.get_t(i));
			
		de::constraints_ptr constraints( boost::make_shared< de::constraints >(m_redundancy, -1.0e6, 1.0e6));
		for (int j = 0; j < m_redundancy; j++) {
			std::pair<double, double> tmp = (m_states.back()).get_range(j);
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

		std::cout << best->cost() << endl;

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
		
		std::cout << cur_state.to_string() << std::endl;

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
		if (cur_state.push_value()) {
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
		std::cout << std::endl;
	}

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
