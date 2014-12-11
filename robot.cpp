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
	void path_plan() {
		for (int i = 0; i < m_job.get_size(); i++) {
			T cur_state(m_job.get_p(i), m_job.get_n(i), m_job.get_t(i));
			
			de::constraints_ptr constraints( boost::make_shared< de::constraints >(m_redundancy, -1.0e6, 1.0e6));
			for (int j = 0; j < m_redundancy; j++) {
				std::pair<double, double> tmp = (m_states.back()).get_range(j);
				(*constraints)[j] = boost::make_shared<de::real_constraint>(tmp.first, tmp.second);
			}

			de::listener_ptr listener( boost::make_shared< de::null_listener >() );
			de::processor_listener_ptr processor_listener( 
				boost::make_shared< de::null_processor_listener >() );
			typename de::processors< T >::processors_ptr _processors( 
				boost::make_shared< de::processors< T > >( 
					m_thread_nr, boost::ref( cur_state ), processor_listener ) );
			de::termination_strategy_ptr terminationStrategy( 
				boost::make_shared< de::max_gen_termination_strategy >( 200 ) );
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
		}
	}

	void set_de_args(int pop_size, int thread_nr, double weight, double crossover) {
		assert(pop_size > 0 && thread_nr > 0 && weight > 0.0 && crossover > 0.0);
		m_pop_size = pop_size;
		m_thread_nr = thread_nr;
		m_weight = weight;
		m_crossover = crossover;
	}
	
	virtual ~robot_system();
};

