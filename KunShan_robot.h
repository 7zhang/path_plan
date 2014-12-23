#ifndef _KUNSHAN_ROBOT_H_
#define _KUNSHAN_ROBOT_H_

#include "system_state.h"

class kunshan_robot: public system_state
{
	IKinematicAlg *rob;

public:
kunshan_robot():system_state(){ rob = new KunshanRKA(); }
	static void init();
	double operator() (de::DVectorPtr args);
};

#endif /* _KUNSHAN_ROBOT_H_ */
