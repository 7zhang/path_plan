#ifndef _CALC_CRITERIA_H_
#define _CALC_CRITERIA_H_

#include "KunShanJacobi.h"
#include "state.h"
#include "calc_criteria.h"

double calc_criteria(JAngle *angle)
{
	double j[6][6];
	jacobi(j, angle->get_angle(1), angle->get_angle(2), angle->get_angle(3), 
		angle->get_angle(4), angle->get_angle(5), angle->get_angle(6));
		
	return fabs(determinant(j));
}

#endif /* _CALC_CRITERIA_H_ */
