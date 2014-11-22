#ifndef _POSITIONER_H_
#define _POSITIONER_H_

#include "volume.h"
#include <math.h>
#include "robotdata.h"

#define PI 3.1415926535897932384626433832795
#define DEGREE_TO_RADIAN (PI / 180)

typedef struct _solution {
	double theta1;
	double theta2;
}solution;

typedef struct _solution_array {
	int num;
	solution *sol;
}solution_array;

//limit between -pi to pi
typedef struct _limit {
	double min;
	double max;
	double step;
}limit;

static inline void free_solution(solution_array *sa) {
	if(sa->sol != NULL) {
		free(sa->sol);
	}
	free(sa);
}

static inline int equal(double lhs, double rhs)
{
	return fabs(lhs - rhs) < 1e-6;
}

static inline int check_limit(double c, limit *lim)
{
	return (c > lim->min - 1e-6) && (c < lim->max + 1e-6);
}
// 
// static inline void vectordot(const vector3d *v1, const vector3d *v2, double *dot)
// {
// 	*dot = v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
// }

static inline double length(const vector3d *v)
{
	double tmp;
	vectordot(v, v, &tmp);
	return sqrt(tmp);
}

static inline double equation2(double x1, double x2, double y1, double y3, double theta1, double theta2)
{
	return fabs(sin(theta2) * x1 + cos(theta2) * x2 - cos(theta1) * y1 - sin(theta1) * y3);
}

//solve equation x1 cos(t) - x2 sin(t) = y
//return the number of solution
int solve_trig_equation(double x1, double x2, double y, double *res1, double *res2);

solution_array *positioner_inverse(vector3d *before, vector3d *after, 
				   limit *lim1, limit *lim2, JAngle& pre_angle);



#endif /* _POSITIONER_H_ */
