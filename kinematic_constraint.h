#ifndef _KINEMATIC_CONSTRAINT_H_
#define _KINEMATIC_CONSTRAINT_H_

using namespace de;
class kinematic_constraint {
private:
	real_constraint m_value;
	double m_max_velocity;
	double m_max_acc;
public:
	kinematic_constraint(real_constraint& value, double& velocity, double& acc);
	constraint_ptr get_real_constraint (double ppre, double pre, double time_interval);	
};

#endif /* _KINEMATIC_CONSTRAINT_H_ */
