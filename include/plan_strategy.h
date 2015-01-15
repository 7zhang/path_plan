#ifndef _PLAN_STRATEGY_H_
#define _PLAN_STRATEGY_H_

#include <vector>
#include <boost/thread.hpp>
#include "robot.h"
#include "geometric.h"

template <typename T>
class plan_strategy
{
	std::vector<Vector3D> m_p;
	std::vector<Vector3D> m_n;
	std::vector<Vector3D> m_t;
	int m_size;
public:
	plan_strategy(std::vector<Vector3D>& p, std::vector<Vector3D>& n, std::vector<Vector3D>& t) : m_p(p), m_n(n), m_t(t) { m_size = p.size();}
	void operator() ();
};

const int try_times = 20;
template <typename T>
void plan_strategy<T>::operator() ()
{
	std::vector<std::string> stl_path;
	std::vector<Vector3D> tmpp, tmpn, tmpt;
	tmpp.push_back(m_p[0]);
	tmpn.push_back(m_n[0]);
	tmpt.push_back(m_t[0]);
	job start_point(tmpp, tmpn, tmpt);;

	//std::vector<Vector3D> p, n, t;
	// for (int i = 0; i < m_size; i += m_size / 10) {
	// 	sample.get_p().push_back(m_p[i]);
	// 	sample.get_n().push_back(m_n[i]);
	// 	sample.get_t().push_back(m_t[i]);
	// }
	
	std::vector<robot_system<T> *> start_tries;
	for (int i = 0; i < try_times; i++) {
		boost::thread_group m_threads;
		for (int i = 0; i < 4; i++) {
			robot_system<T> *work = new robot_system<T>(i, 60, 0.001, stl_path, start_point);
			start_tries.push_back(work);
			boost::thread* th( new boost::thread(boost::ref(*work)) );
			m_threads.add_thread( th );
		}
		m_threads.join_all();
	}

//	m_threads.join_all();

	std::vector<robot_system<T> *> candidate;

	candidate.push_back(start_tries[0]);
//	std::cout << "candidate size: " << candidate.size() << std::endl;
//	std::cout << "start_tries size: " << start_tries.size() << std::endl;
	for (int i = 1; i < try_times; i++) {
		int result = 1;
//		std::cerr << std::endl << start_tries[i]->m_states[0].to_string() << std::endl;
//		std::cerr << "start_tries[i]->m_states.size(): " << (start_tries[i]->m_states).size() << std::endl;
		for (int j = 0; j < candidate.size(); j++) {
//			std::cerr << "candidate.size(): " << candidate.size() << std::endl; 

			double d = (candidate[j]->m_states)[0].dist((start_tries[i]->m_states)[0]);
//			std::cout << "d = " << d << std::endl;
			result = result && (start_tries[i]->recommend > 0.0) && (d > 10);
			/* if (!result) */
			/* 	break; */
		}
		if (result) {
//			std::cout << "inserted" << std::endl;
			candidate.push_back(start_tries[i]);
		} else {
//			std::cout << "not inserted" << std::endl;
			delete start_tries[i];
		}
	}

	std::cerr << std::endl << "candidate size: " << candidate.size() << std::endl;
	for (int i = 0; i < candidate.size(); i++) {
		std::cerr << std::endl << candidate[i]->m_states[0].to_string() << std::endl;
	}
	if (candidate.size() == 0) {
		return ;
	}

	tmpp.clear();
	tmpn.clear();
	tmpt.clear();
	for (int i = 0; i < m_size; i += 10) {
		tmpp.push_back(m_p[i]);
		tmpn.push_back(m_n[i]);
		tmpt.push_back(m_t[i]);
	}

	job sample(tmpp, tmpn, tmpt);
	std::vector<robot_system<T> *> tries;
	boost::thread_group ths1;
	std::cerr << "sample size: " << sample.get_size() << std::endl;
	for (int i = 0; i < candidate.size(); i++) {
		try {
			robot_system<T> *work = new robot_system<T>(i, 60, 0.001, stl_path, sample);
			work->push_value(candidate[i]->m_states[0]);
			tries.push_back(work);
			boost::thread* th( new boost::thread(boost::ref(*work)) );
			ths1.add_thread( th );
		} catch (...) {
			std::cerr << "exception" << std::endl;
		}
	}

	ths1.join_all();

	/* std::cerr << "finished" << std::endl; */
	int index = 0;
	double max_recommend = -1.0;
	for (int i = 0; i < tries.size(); i++) {
		if (tries[i]->recommend > max_recommend) {
			max_recommend = tries[i]->recommend;
			index = i;
		}
	}

	if (max_recommend < 0.0) {
		std::cerr << "can't finish" << std::endl;
		return;
	}
// now finish the whole plan with start point at tries[index]
	boost::thread_group ths2;
	robot_system<T> *work = new robot_system<T>(0, 60, 0.001, stl_path, job(m_p, m_n, m_t));
	work->push_value(tries[index]->m_states[0]);
	tries.push_back(work);
	boost::thread* th( new boost::thread(boost::ref(*work)) );
	ths2.add_thread( th );
	
	ths2.join_all();

	std::cerr << "globally best path index: " << index << std::endl;
	std::cerr << std::endl << "start point:" << tries[index]->m_states[0].to_string() << std::endl;	
}

#endif /* _PLAN_STRATEGY_H_ */
