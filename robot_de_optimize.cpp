#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <limits>
#include <string>
#include <cmath>
#include "differential_evolution.hpp"
#include "load_seam.h"
#include "state.h"
#include "calc_criteria.h"

using namespace de;

#define DIMENSION 6
class robot_optimize_function {
private:
	const std::string m_name;
	state m_s;
	state m_pre_s;
	double limit_min[DIMENSION];
	double limit_max[DIMENSION];
	double mu[DIMENSION];
	double sigma[DIMENSION];
	
public:
	robot_optimize_function(const std::string &name, state &s, state &pre_s)
		: m_name(name), m_s(s), m_pre_s(pre_s)
		{
			limit_min[0] = -150.0;
			limit_min[1] = -125.0;
			limit_min[2] = -120.0;
			limit_min[3] = -180.0;
			limit_min[4] = -120.0;
			limit_min[5] = -180.0;
			
			limit_max[0] = 180.0;
			limit_max[1] = 30.0;
			limit_max[2] = 150.0;
			limit_max[3] = 180.0;
			limit_max[4] = 120.0;
			limit_max[5] = 180.0;
			// for (int i = 0; i < DIMENSION; i++) {
			// 	limit_min[i] = m_pre_s.angle.angle[i] - 10;
			// 	limit_max[i] = m_pre_s.angle.angle[i] + 10;
			// }

			for (int i = 0; i < DIMENSION; i++) {
				mu[i] = (limit_min[i] + limit_max[i]) / 2;
				sigma[i] = (limit_max[i] - limit_min[i]) / 6;
			}
			m_s.m_cri.resize(2);
		}
	double operator() (de::DVectorPtr args)
		{
			int err;
			m_s.in.x = (*args)[0];
			m_s.in.theta = (*args)[1];
			m_s.in.pthai = (*args)[2];
			m_s.in.fai1 = (*args)[3];
			m_s.in.fai2 = (*args)[4];
			double ret = obj_function(&m_s, &m_pre_s, &err);
			ret = ret / 376234706.2853961;
			if (err) {
				return 0;
				return -1e6;
//				return std::numeric_limits<double>::quiet_NaN();
			} else {
				double c2 = calc1();
//				std::cout << "c2 = " << c2 / DIMENSION << endl;

				m_s.m_cri[0] = ret;
				m_s.m_cri[1] = c2;
//				return ret;
				return 0.5 * ret + 0.5 * c2;
			}
		}
	double calc1() {
		double sum = 0;

		for (int i = 0; i < DIMENSION; i++) {
//			std::cout << m_s.angle.angle[i] << " " << mu[i] << " " << sigma[i] << endl;
			double tmp = m_s.angle.angle[i] - mu[i];
			sum += exp(- tmp * tmp / 2 / sigma[i] / sigma[i]);
		}

//		std::cout << "sum = " << sum << endl;
		return sum / DIMENSION;
//		return exp(-sum);
	}

//	const std::vector<double>& cri() { return m_cri; }
	const std::string& name() const { return m_name; }
	const state& get_state() const { return m_s; }
};

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

#define VARS_COUNT 5
#define POPULATION_SIZE (VARS_COUNT * 10)

int robot_path()
{
	try {
		state s, pre_s;
		s.in.lim[6].max = 91.0 * DEGREE_TO_RADIAN;
		s.in.lim[6].min = -10.0 * DEGREE_TO_RADIAN;
		s.in.lim[6].step = 1.0 * DEGREE_TO_RADIAN;

		s.in.lim[7].max = 180.0 * DEGREE_TO_RADIAN;
		s.in.lim[7].min = -180 * DEGREE_TO_RADIAN;
		s.in.lim[7].step = 1.0 * DEGREE_TO_RADIAN;
		s.in.x = 0.0;
		s.in.theta = 0;
		s.in.pthai = 0;
		s.in.fai1 = 0.0;
		s.in.fai2 = 0;
		pre_s.angle.set_angles(0.0, -90.00, 90.00, 0.0, 0.0, 0.0);
		pre_s.ex_angle.set_angles(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

		std::vector<Vector3D> normal, tangent, point;
		if (load_seam("test1.pos", point, normal, tangent)) {
			printf("load_seam error\n");
			return -1;
		}

		// Vector3D normal(-0.000000, -0.707107, 0.707107);
		// Vector3D tangent(-1.000000, 0.000000, 0.000000);
		// Vector3D point(-250.000000, 500.000000, 80.000000);
		for (int i = 0; i < normal.size(); i++) {
			s.in.n = normal[i];
			s.in.t = tangent[i];
			s.in.p = point[i];

			constraints_ptr constraints( boost::make_shared< constraints >(VARS_COUNT, -1.0e6, 1.0e6));

			if (i != 0) {
				(*constraints)[0] = boost::make_shared<real_constraint>(pre_s.in.x - 10, pre_s.in.x + 10);
				(*constraints)[1] = boost::make_shared<real_constraint>(pre_s.in.theta - 10, pre_s.in.theta + 10);
				(*constraints)[2] = boost::make_shared<real_constraint>(pre_s.in.pthai - 10, pre_s.in.pthai + 10);
				(*constraints)[3] = boost::make_shared<real_constraint>(pre_s.in.fai1 - 10, pre_s.in.fai1 + 10);
				(*constraints)[4] = boost::make_shared<real_constraint>(pre_s.in.fai2 - 10, pre_s.in.fai2 + 10);
			} else {
				(*constraints)[0] = boost::make_shared<real_constraint>(-1000, 1000);
				(*constraints)[1] = boost::make_shared<real_constraint>(-180, 180);
				(*constraints)[2] = boost::make_shared<real_constraint>(-180, 180);
				(*constraints)[3] = boost::make_shared<real_constraint>(-180, 180);
				(*constraints)[4] = boost::make_shared<real_constraint>(-180, 180);
			}
			robot_optimize_function of(std::string("robot_optimize_function"), s, pre_s);
			listener_ptr listener( boost::make_shared< null_listener >() );
			processor_listener_ptr processor_listener( 
				boost::make_shared< null_processor_listener >() );
			processors< robot_optimize_function >::processors_ptr _processors( 
				boost::make_shared< processors< robot_optimize_function > >( 
					1, boost::ref( of ), processor_listener ) );
			termination_strategy_ptr terminationStrategy( 
				boost::make_shared< max_gen_termination_strategy >( 2000 ) );
			selection_strategy_ptr selectionStrategy(
				boost::make_shared< best_parent_child_selection_strategy >() );
			mutation_strategy_arguments mutation_arguments( 0.8, 0.9 );
			mutation_strategy_ptr mutationStrategy(
				boost::make_shared< mutation_strategy_1 >( VARS_COUNT, mutation_arguments ) );
			differential_evolution< robot_optimize_function > de(
				VARS_COUNT, POPULATION_SIZE, _processors, constraints, false, 
				terminationStrategy, selectionStrategy, mutationStrategy, listener );

			de.run();
			individual_ptr best = de.best();

			// std::cout << "maximium value for the " << of.name() << " is " 
			// 	  << best->cost() << std::endl;
			std::cout << best->to_string() << " ";
			//cout << of(best->vars()) << endl;
//			of(best->vars());
//			state best_state = of.get_state();
			state best_state = best->get_state();
//			std::vector<double>& cri = of.cri();
			print_state(best_state);
			
//			std::cout << "robot_optimize_function s: ";
//			print_state(s);
			pre_s = best_state;
			// std::cout << "robot_optimize_function s: ";
			// print_state(pre_s);

			std::cerr << i << endl;
		}
		return 0;
	} catch (const de::exception &e)
	{
		std::cout << "an error occurred: " << e.what();
	}
}

int main(int argc, char *argv[])
{
	robot_path();
	return 0;
}
