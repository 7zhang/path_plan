#include "differential_evolution.hpp"
#include "kinematic_constraint.h"

kinematic_constraint::kinematic_constraint(real_constraint& value, double& velocity, double& acc)
	:m_value(value), m_max_velocity(velocity), m_max_acc(acc)
{
	assert(value > 0 && velocity > 0 && acc > 0);
}
constraint_ptr kinematic_constraint::get_real_constraint (double ppre, 
							  double pre, double time_interval)
{
	double cur_vel = (pre - ppre) / time_interval;
	double max_vel = cur_vel + m_max_acc.max() * time_interval;
	double min_vel = cur_vel - m_max_acc.max() * time_interval;

	assert(m_max_vel >= m_min_vel);

	if (max_vel > m_max_velocity) {
		max_vel = m_max_velocity;
	} else if (max_vel < - m_max_velocity) {
		max_vel = - m_max_velocity;
	}

	if (min_vel > m_max_velocity) {
		min_vel = m_max_velocity;
	} else if (min_vel < - m_max_velocity) {
		min_vel = - m_max_velocity;
	}
		
	double max_value = pre + max_vel * time_interval;
	double min_value = pre + min_vel * time_interval;
		
	if (max_value > m_value.max()) {
		max_value = m_value.max();
	} else if (max_value < m_value.min()) {
		max_value = m_value.min();
	}

	if (min_value > m_value.max()) {
		min_value = m_value.max();
	} else if (min_value < m_value.min()) {
		min_value = m_value.min();
	}

	return boost::make_shared<real_constraint>(min_value, max_value);
}
