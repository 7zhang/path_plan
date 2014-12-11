#include "axis.h"
#include <limits>
#include <utility>

double axis::operator() (int t) {
	if (m_c) {
		return (*m_c)(wave[t]);
	} else {
		return std::numeric_limits<double>::quiet_NaN();
	}
}

std::pair<double, double> axis::get_range () {
	double max;
	double min;

	int index = wave.size() - 1;
	double cur_value = wave[index];
	double cur_vel;
	if (index < 1) {
		cur_vel = 0.0;
	} else {
		cur_vel = (wave[index] - wave[index - 1]) / m_time_interval;
	}

	assert(fabs(cur_vel) < m_vel_max);
	double t1 = (m_vel_max - cur_vel) / m_acc_max;
	if (t1 > m_time_interval) {
		max = cur_value + cur_vel * m_time_interval
			+ m_acc_max * m_time_interval * m_time_interval / 2.0;
	} else {
		max = cur_value + cur_vel * t1 + m_acc_max * t1 * t1 / 2.0
			+ m_vel_max * (m_time_interval - t1);
	}

	double t2 = (m_vel_max + cur_vel) / m_acc_max;
	if (t2 > m_time_interval) {
		min = cur_value + cur_vel * m_time_interval
			- m_acc_max * m_time_interval * m_time_interval / 2.0;
	} else {
		min = cur_value + cur_vel * t2 - m_acc_max * t2 * t2 / 2.0
			- m_vel_max * (m_time_interval - t2);
	}

	return std::pair<double, double>(std::max(m_min, min), std::min(m_max, max));
}

void axis::add_value (double value)
{
	assert(value <= m_max && value >= m_min);
	wave.push_back(value);
}
