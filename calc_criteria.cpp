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

#endif /* _CALC_CRITERIA_H_ */
