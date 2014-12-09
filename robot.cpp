#include <iostream>
#include <vector>
#include <string>

#include "differential_evolution.hpp"
#include "axis.h"
#include "geometric.h"
#include "load_seam.h"

class weld_system_state;

class weld_job
{
private:
	int m_size;
	std::vector<Vector3D> m_normal, m_tangent, m_point;
public:
	weld_job(std::string seam) {
		if (load_seam(seam.c_str(), m_point, m_normal, m_tangent)) {
			std::cout << "load_seam error: " << seam << std::endl;
		}

		m_size = m_normal.size();
	};
	
	const Vector3D& get_n(int index);
	const Vector3D& get_t(int index);
	const Vector3D& get_p(int index);

	int get_size() { return m_size; }

	virtual ~weld_job();
};

class weld_system
{
private:
	std::string m_sys_name;
	int m_redundancy;
	int m_time_interval;

	std::vector<weld_system_state> m_states;
	std::vector<std::string> m_stl_path;
//	std::string m_seam;
	weld_job m_job;
public:
	weld_system(std::string sys_name, int redundancy, int time_interval, 
		    std::string stl_path, std::string seam):
		m_sys_name(sys_name), m_redundancy(redundancy),
		m_time_interval(time_interval), m_stl_path(stl_path),
		m_job(seam) {}
	void path_plan() {
		for (int i = 0; i < m_job.get_size(); i++) {
			weld_system_state cur_state(m_job.get_p(i),
						    m_job.get_n(i),
						    m_job.get_t(i));
			constraints_ptr constraints( boost::make_shared< constraints >(m_redundancy, -1.0e6, 1.0e6));
			for (int j = 0; j < m_redundancy; j++) {
				(*constraints)[j] = boost::make_shared<real_constraint>(get_range(j));
			}

			listener_ptr listener( boost::make_shared< null_listener >() );
			processor_listener_ptr processor_listener( 
				boost::make_shared< null_processor_listener >() );
			processors< robot_optimize_function >::processors_ptr _processors( 
				boost::make_shared< processors< robot_optimize_function > >( 
					4, boost::ref( cur_state ), processor_listener ) );
			termination_strategy_ptr terminationStrategy( 
				boost::make_shared< max_gen_termination_strategy >( 200 ) );
			selection_strategy_ptr selectionStrategy(
				boost::make_shared< tournament_selection_strategy >() );
			mutation_strategy_arguments mutation_arguments( 0.8, 0.9 );
			mutation_strategy_ptr mutationStrategy(
				boost::make_shared< mutation_strategy_1 >( VARS_COUNT, mutation_arguments ) );
			differential_evolution< weld_system_state > de(
				VARS_COUNT, POPULATION_SIZE, _processors, constraints, false, 
				terminationStrategy, selectionStrategy, mutationStrategy, listener );

			de.run();
			individual_ptr best = de.best();
		}
	};

	
	virtual ~weld_system();
};

class weld_system_state
{
private:
	Vector3D& m_p;
	Vector3D& m_n;
	Vector3D& m_t;

	std::vector<axis> m_axes;
	std::vector<axis> m_auxiliary_variables;

	std::vector<double> m_sub_cris;
	double m_cri;
public:
	weld_system_state(Vector3D& p, Vector3D& n, Vector3D& t)
		: m_p(p), m_n(n), m_t(t) {}
	double operator() (de::DVectorPtr args);
	virtual ~weld_system_state();
};
