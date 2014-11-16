#ifndef _CALC_CRITERIA_H_
#define _CALC_CRITERIA_H_

#include "KunShanJacobi.h"
#include "state.h"
#include "calc_criteria.h"

int calc_criteria(state *s, criteria *c)
{
	input *inp = &s->in;
	double j[6][6];
	jacobi(j, s->angle.get_angle(1), s->angle.get_angle(2), s->angle.get_angle(3), 
		s->angle.get_angle(4), s->angle.get_angle(5), s->angle.get_angle(6));
		
	c->j = fabs(determinant(j));

	return 0;
}

double obj_function(state *s, state *pre_s, int *err)
{
	if (calc_state(s, pre_s)) {
		*err = -1;
		return 0;
	} else {
		criteria c;
		to_continuous(s->angle, pre_s->angle);
		to_continuous(s->ex_angle, pre_s->ex_angle);
		calc_criteria(s, &c);
		// std::cout << "s after calc_criteria: "; 
		// print_state(*s);
		*err = 0;
		return c.j;
	}
}

#endif /* _CALC_CRITERIA_H_ */
