#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <string>
#include <cmath>
#include "differential_evolution.hpp"
#include "kinematic_constraint.h"
#include "load_seam.h"
#include "state.h"
#include "calc_criteria.h"

using namespace de;

int program_jpos(vector<JAngle> &angle, vector<JAngle> &ex_angle, char *path);
int program_cpos(vector<RPY> &rpy, vector<JAngle> &ex_angle, char *path);

#define DIMENSION 9
class robot_optimize_function {
private:
	const std::string m_name;
	state m_s;
	state m_pre_s;
	double limit_min[DIMENSION];
	double limit_max[DIMENSION];
	double mu[DIMENSION];
	double sigma[DIMENSION];
	double m_power[DIMENSION];
	
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
			limit_min[6] = 0.0;
			limit_min[7] = -180.0;
			limit_min[8] = -1700.0;
			
			limit_max[0] = 180.0;
			limit_max[1] = 30.0;
			limit_max[2] = 150.0;
			limit_max[3] = 180.0;
			limit_max[4] = 120.0;
			limit_max[5] = 180.0;
			limit_max[6] = 90.0;
			limit_max[7] = 180;
			limit_max[8] = 1600.0;
			// for (int i = 0; i < DIMENSION; i++) {
			// 	limit_min[i] = m_pre_s.angle.angle[i] - 10;
			// 	limit_max[i] = m_pre_s.angle.angle[i] + 10;
			// }

			for (int i = 0; i < DIMENSION; i++) {
				mu[i] = (limit_min[i] + limit_max[i]) / 2;
				sigma[i] = (limit_max[i] - limit_min[i]) / 6;
			}
			m_s.m_cri.resize(6);
			m_power[0] = 1;
			m_power[1] = 1;
			m_power[2] = 1;
			m_power[3] = 1;
			m_power[4] = 1;
			m_power[5] = 1;
			m_power[6] = 1;
			m_power[7] = 1;
			m_power[8] = 1;
			
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
				// return 0;
				// return -1e6;

//				std::cout << "NAN returned" << std::endl;
				return std::numeric_limits<double>::quiet_NaN();
			} else {
				double c1 = calc1();
				double c4 = calc4();
//				std::cout << "c2 = " << c2 / DIMENSION << endl;

				m_s.m_cri[0] = ret;
				m_s.m_cri[1] = c1;
				m_s.m_cri[2] = c4;
				return ret * c1 * c4;
//				return c2;
//				return ret;
			}
		}
	double calc1() {
		double sum = 0;

		for (int i = 0; i < DIMENSION; i++) {
			double tmp = m_s.angle.angle[i] - mu[i];
			sum += exp(- tmp * tmp / 2 / sigma[i] / sigma[i]);
		}

		return sum / DIMENSION;
	}

	double calc2() {
		double sum = 0;

		for (int i = 0; i < DIMENSION; i++) {
			double tmp = m_s.angle.angle[i] - mu[i];
			sum += - tmp * tmp / 2 / sigma[i] / sigma[i];
		}

		return 1 + sum;
	}

	double calc3() {
		double sum = 0;

		for (int i = 0; i < DIMENSION; i++) {
			double tmp = m_s.angle.angle[i] - mu[i];
			double l = sigma[i] * 3;
			double t = tmp / l;
			sum += 1 / (t * t - 1);
		}

		return 1 + sum / DIMENSION;
	}

	double calc4() {
		if (!in_range()) {
			return 0.0;
		}

		double product = 1;
		double pi = boost::math::constants::pi<double>();

//		std::cout << "angle = " << std::endl;
//		print_state(m_s);

		for (int i = 0; i < 6; i++) {
			double l = 6 * sigma[i];
			double tmp = m_s.angle.angle[i] - mu[i];
			double cosine = cos(tmp / l * pi);
			cosine = pow(cosine, m_power[i]);
//			std::cout << "cosine = " << cosine << std::endl;
			product *= cosine;
		}

		for (int i = 0; i < 3; i++) {
			double l = 6 * sigma[i + 6];
			double tmp = m_s.ex_angle.angle[i] - mu[i + 6];
			double cosine = cos(tmp / l * pi);
			cosine = pow(cosine, m_power[i + 6]);
//			std::cout << "cosine = " << cosine << std::endl;
			product *= cosine;
		}

//		std::cout << "product = " << product << std::endl;
		return product;
	}

	bool in_range() {
		for (int i = 0; i < 6; i++) {
			double tmp = m_s.angle.angle[i];
			if (tmp < limit_min[i] || tmp > limit_max[i]) {
				return false;
			}
		}

		for (int i = 0; i < 3; i++) {
			double tmp = m_s.ex_angle.angle[i];
			if (tmp < limit_min[i + 6] || tmp > limit_max[i + 6]) {
				return false;
			}
		}

		return true;
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

//		s.in.lim[7].max = 360.0 * DEGREE_TO_RADIAN;
//		s.in.lim[7].min = -360 * DEGREE_TO_RADIAN;

		s.in.lim[7].max = 180.0 * DEGREE_TO_RADIAN;
		s.in.lim[7].min = -180.0 * DEGREE_TO_RADIAN;
		s.in.lim[7].step = 1.0 * DEGREE_TO_RADIAN;
		s.in.x = 0.0;
		s.in.theta = 0;
		s.in.pthai = 0;
		s.in.fai1 = 0.0;
		s.in.fai2 = 0;
		pre_s.angle.set_angles(15.0, -95.0/2, 15.0, 0.0, 0.0, 0.0);
		pre_s.ex_angle.set_angles(45.0, 0.0, -50.0, 0.0, 0.0, 0.0);

		std::vector<Vector3D> normal, tangent, point;
		if (load_seam("test1.pos", point, normal, tangent)) {
			printf("load_seam error\n");
			return -1;
		}

		// Vector3D normal(-0.000000, -0.707107, 0.707107);
		// Vector3D tangent(-1.000000, 0.000000, 0.000000);
		// Vector3D point(-250.000000, 500.000000, 80.000000);

		vector<JAngle> best_angle;
		vector<JAngle> best_ex_angle;

		int err_count = 0;
		
		for (int i = 0; i < normal.size(); i++) {
//			std::cout << "i = " << i << std::endl;
		  //for (int i = 0; i < 1; i++) {
			s.in.n = normal[i];
			s.in.t = tangent[i];
			s.in.p = point[i];

			constraints_ptr constraints( boost::make_shared< constraints >(VARS_COUNT, -1.0e6, 1.0e6));

			if (i != 0) {
				(*constraints)[0] = boost::make_shared<real_constraint>(std::max(-1700.0, pre_s.in.x - 10), 
							                                std::min(1600.0, pre_s.in.x + 10));
				(*constraints)[1] = boost::make_shared<real_constraint>(pre_s.in.theta - 10, 
											pre_s.in.theta + 10);
				(*constraints)[2] = boost::make_shared<real_constraint>(std::max(-45.0, pre_s.in.pthai - 10),
											std::min(45.0, pre_s.in.pthai + 10));
				(*constraints)[3] = boost::make_shared<real_constraint>(pre_s.in.fai1 - 10, 
						                                        pre_s.in.fai1 + 10);
				(*constraints)[4] = boost::make_shared<real_constraint>(pre_s.in.fai2 - 10, 
						                                        pre_s.in.fai2 + 10);
			} else {
				(*constraints)[0] = boost::make_shared<real_constraint>(-1700, 1600);
				(*constraints)[1] = boost::make_shared<real_constraint>(-180, 180);
				(*constraints)[2] = boost::make_shared<real_constraint>(-45, 45);
				(*constraints)[3] = boost::make_shared<real_constraint>(-90, 90);
				(*constraints)[4] = boost::make_shared<real_constraint>(-90, 90);

				// (*constraints)[0] = boost::make_shared<real_constraint>(-1000, 1000);
				// (*constraints)[1] = boost::make_shared<real_constraint>(-180, 180);
				// (*constraints)[2] = boost::make_shared<real_constraint>(-45, 45);
				// (*constraints)[3] = boost::make_shared<real_constraint>(-90, 90);
				// (*constraints)[4] = boost::make_shared<real_constraint>(-90, 90);
			}
			robot_optimize_function of(std::string("robot_optimize_function"), s, pre_s);
			listener_ptr listener( boost::make_shared< null_listener >() );
			processor_listener_ptr processor_listener( 
				boost::make_shared< null_processor_listener >() );
			processors< robot_optimize_function >::processors_ptr _processors( 
				boost::make_shared< processors< robot_optimize_function > >( 
					4, boost::ref( of ), processor_listener ) );
			termination_strategy_ptr terminationStrategy( 
				boost::make_shared< max_gen_termination_strategy >( 200 ) );
			selection_strategy_ptr selectionStrategy(
				boost::make_shared< tournament_selection_strategy >() );
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

			//cout << of(best->vars()) << endl;
//			of(best->vars());
//			state best_state = of.get_state();
			state best_state = best->get_state();

			double diff = dist(best_state, pre_s);
//			std::cout << "diff = " << diff << std::endl;
			if (i > 0 && diff > 20 && err_count < 10) {
				err_count++;
				i--;
				continue;
			}

			if (err_count > 9) {
				std::cerr << "failled but tried" << std::endl;
			}
			err_count = 0;
			std::cout << best->to_string() << " ";
			best_angle.push_back(best_state.angle);
			best_ex_angle.push_back(best_state.ex_angle);
//			std::vector<double>& cri = of.cri();
//			to_continuous(best_state.angle, pre_s.angle);
			print_state(best_state);
			
//			std::cout << "robot_optimize_function s: ";
//			print_state(s);

			pre_s = best_state;
			// std::cout << "robot_optimize_function s: ";
			// print_state(pre_s);

//			std::cout << "i = " << i << std::endl;
			std::cerr << i << endl;
		}

		program_jpos(best_angle, best_ex_angle, "./program.glp");
		return 0;
	} catch (const de::exception &e)
	{
		std::cout << "an error occurred: " << e.what();
	}
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

int main(int argc, char *argv[])
{
	robot_path();
	return 0;
}
