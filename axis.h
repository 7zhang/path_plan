#ifndef _AXIS_H_
#define _AXIS_H_

#include <vector>
#include "sine_criteria.h"
#include "exp_criteria.h"

class axis
{
private:
	double m_min;
	double m_max;
	double m_vel_max;
	double m_acc_max;
	double m_time_interval;
	
	std::vector<double> wave;
	cri *m_c;
public:
axis(double min, double max, double vel_max, double acc_max, double time_interval, int c_type, double para)
	: m_min(min), m_max(max), m_vel_max(vel_max), m_acc_max(acc_max), m_time_interval(time_interval) {
		assert(min < max);
		assert(vel_max > 0.0);
		assert(acc_max > 0.0);
		assert(time_interval > 1e-6);

		switch (c_type) {
		case 0:
			m_c = new sine_criteria(m_min, m_max, para);
			break;
		case 1:
			m_c = new exp_criteria((m_min + m_max) / 2.0, (m_max - m_min) / 6.0, para);
			break;
		default:
			m_c = NULL;
			break;
		}
	}
	axis(){}

	double operator() (double value) const;
	double mid() const { return (m_min + m_max) / 2.0; }
	double length() const { return m_max - m_min; }
	std::pair<double, double> get_range() const;
	int add_value(double value);
	double last() const;
};

#endif /* _AXIS_H_ */
