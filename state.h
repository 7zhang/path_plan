#ifndef __STATE_H__
#define __STATE_H__

#include <iostream>
#include <iomanip>
#include "Transform.h"
#include "geom.h"
#include "positioner.h"

typedef struct input {
	Vector3D p;
	Vector3D n;
	Vector3D t;

	/*
	 * var[0] x
	 * var[1] theta
	 * var[2] pthai
	 * var[3] fai1
	 * var[4] fai2
	 */
	double x;
	double theta;
	double pthai;
	double fai1;
	double fai2;

	/*
	 * limit[0] angle[0]
	 * limit[1] angle[1]
	 * limit[2] angle[2]
	 * limit[3] angle[3]
	 * limit[4] angle[4]
	 * limit[5] angle[5]
	 * limit[6] ex_angle[0]
	 * limit[7] ex_angle[1]
	 * limit[8] ex_angle[2]
	 */
	limit lim[9];
} input;

typedef struct criteria {
	double j;
} criteria;

typedef struct state {
	input in;
	int collision;
	/*
	 * robot angle
	 */
	JAngle angle;

	/*
	 * ex_angle[0] positioner 0
	 * ex_angle[1] positioner 1
	 * ex_angle[2] rail
	 */
	JAngle ex_angle;

	criteria c;
	std::vector<double> m_cri;
} state;

static inline void print_state(state& s)
{
	std::cout << std::setw(10) << s.angle.get_angle(1) << " "
		  << std::setw(10) << s.angle.get_angle(2) << " "
		  << std::setw(10) << s.angle.get_angle(3) << " "
		  << std::setw(10) << s.angle.get_angle(4) << " "
		  << std::setw(10) << s.angle.get_angle(5) << " "
		  << std::setw(10) << s.angle.get_angle(6) << " "
		  << std::setw(10) << s.ex_angle.get_angle(1) << " "
		  << std::setw(10) << s.ex_angle.get_angle(2) << " "
		  << std::setw(10) << s.ex_angle.get_angle(3) << " "
		  << std::setw(10) << s.m_cri[0] << " "
		  << std::setw(10) << s.m_cri[1] << endl;
}

int calc_state(state *s, state *pre_s);

static inline void to_continuous(JAngle &angle, JAngle &last)
{
	if (angle.get_angle(1) - last.get_angle(1) > 90) {
		angle.set_angle(angle.get_angle(1) - 2 * 180, 1);
	} else if (angle.get_angle(1) - last.get_angle(1) < -90) {
		angle.set_angle(angle.get_angle(1) + 2 * 180, 1);
	}

	if (angle.get_angle(2) - last.get_angle(2) > 90) {
		angle.set_angle(angle.get_angle(2) - 2 * 180, 2);
	} else if (angle.get_angle(2) - last.get_angle(2) < -90) {
		angle.set_angle(angle.get_angle(2) + 2 * 180, 2);
	}

	if (angle.get_angle(3) - last.get_angle(3) > 90) {
		angle.set_angle(angle.get_angle(3) - 2 * 180, 3);
	} else if (angle.get_angle(3) - last.get_angle(3) < -90) {
		angle.set_angle(angle.get_angle(3) + 2 * 180, 3);
	}

	if (angle.get_angle(4) - last.get_angle(4) > 90) {
		angle.set_angle(angle.get_angle(4) - 2 * 180, 4);
	} else if (angle.get_angle(4) - last.get_angle(4) < -90) {
		angle.set_angle(angle.get_angle(4) + 2 * 180, 4);
	}

	if (angle.get_angle(5) - last.get_angle(5) > 90) {
		angle.set_angle(angle.get_angle(5) - 2 * 180, 5);
	} else if (angle.get_angle(5) - last.get_angle(5) < -90) {
		angle.set_angle(angle.get_angle(5) + 2 * 180, 5);
	}

	if (angle.get_angle(6) - last.get_angle(6) > 90) {
		angle.set_angle(angle.get_angle(6) - 2 * 180, 6);
	} else if (angle.get_angle(6) - last.get_angle(6) < -90) {
		angle.set_angle(angle.get_angle(6) + 2 * 180, 6);
	}
}

#endif




