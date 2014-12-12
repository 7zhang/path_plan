#include "robot.h"

template <typename T>
void robot_system<T>::path_plan()
{
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

template <typename T>
void robot_system<T>::set_de_args(int pop_size, int thread_nr, double weight, double crossover) {
	assert(pop_size > 0 && thread_nr > 0 && weight > 0.0 && crossover > 0.0);
	m_pop_size = pop_size;
	m_thread_nr = thread_nr;
	m_weight = weight;
	m_crossover = crossover;
}

