#include <math.h>
#include <stdlib.h>
#include "positioner.h"

static inline double length_between(JAngle& lhs, JAngle& rhs)
{
	double tmp = 0;
	for (int i = 0; i < 6; i++) {
		double diff = lhs.angle[i] - rhs.angle[i];
		tmp += diff * diff;
	}
	
	return sqrt(tmp);
}

//solve equation x1 cos(t) - x2 sin(t) = y
//return 0 if two sol, 1 if infinite sol, -1 if error
int solve_trig_equation(double x1, double x2, double y, double *res1, double *res2)
{
	double r, a, s;
	r = sqrt(x1 * x1 + x2 * x2);
	a = atan2(x2, x1);

	if(r <= 1e-6) {
		if(fabs(y) < 1e-6) {
			return 1;
		} else {
			return -1;
		}
	}
	
	if(r - y < -1e-6) {
		return -1;
	}

	double d;
	d = fabs(r * r - y * y);
	s = sqrt(d);
	*res1 = atan2(s, y) - a;
	*res2 = atan2(-s, y) - a;

	if(*res1 > PI) {
		*res1 -= 2 * PI;
	}

	if(*res1 < -PI) {
		*res1 += 2 * PI;
	}

	if(*res2 > PI) {
		*res2 -= 2 * PI;
	}
	
	if(*res2 < -PI) {
		*res2 += 2 * PI;
	}

	return 0;
}

/* find theta1 and theta2 to satisfy T*before = after, where T is the
 * rotation part of the transformation matrix
 */
solution_array *positioner_inverse(vector3d *before, vector3d *after, 
				   limit *lim1, limit *lim2, JAngle& pre_angle)
{
	double len1, len2;
	len1 = length(before);
	len2 = length(after);
	if (len1 < 1e-6 || len2 < 1e-6) {
		return NULL;
	}
	before->x = before->x / len1;
	before->y = before->y / len1;
	before->z = before->z / len1;
	after->x = after->x / len2;
	after->y = after->y / len2;
	after->z = after->z / len2;
	if(fabs(length(before) - length(after)) >1e-6) {
		return NULL;
	}
	double theta1[2], theta2[2];
	int ret1, ret2;
	ret1 = solve_trig_equation(after->z, after->x, before->z, &theta1[0], &theta1[1]);
	ret2 = solve_trig_equation(before->x, before->y, -after->y, &theta2[0], &theta2[1]);
	
	if(ret1 != -1 && ret2 != -1) {
		int ret = ret1 * 2 + ret2;
		solution_array *sa;
		int i,j;
		switch(ret) {
		case 0:
			//two theta1, two theta2
			sa = (solution_array *)malloc(sizeof(solution_array));
			sa->sol = (solution *)malloc(sizeof(solution) * 4);
			sa->num = 0;
			if(equation2(before->x, before->y, after->x, after->z, theta1[0], theta2[0]) <= 1e-6) {
				if(check_limit(theta1[0], lim1) && check_limit(theta2[0], lim2)) {
					sa->sol[sa->num].theta1 = theta1[0];
					sa->sol[sa->num].theta2 = theta2[0];
					sa->num++;
				}
			}
			if(!equal(theta2[0], theta2[1])) {
				if(equation2(before->x, before->y, after->x, after->z, theta1[0], theta2[1]) <= 1e-6) {
					if(check_limit(theta1[0], lim1) && check_limit(theta2[1], lim2)) {
						sa->sol[sa->num].theta1 = theta1[0];
						sa->sol[sa->num].theta2 = theta2[1];
						sa->num++;
					}
				}
			}

			if(!equal(theta1[0], theta1[1])) {
				if(equation2(before->x, before->y, after->x, after->z, theta1[1], theta2[0]) <= 1e-6) {
					if(check_limit(theta1[1], lim1) && check_limit(theta2[0], lim2)) {
						sa->sol[sa->num].theta1 = theta1[1];
						sa->sol[sa->num].theta2 = theta2[0];
						sa->num++;
					}
				}
				if(!equal(theta2[0], theta2[1])) {
					if(equation2(before->x, before->y, after->x, after->z, theta1[1], theta2[1]) <= 1e-6) {
						if(check_limit(theta1[1], lim1) && check_limit(theta2[1], lim2)) {
							sa->sol[sa->num].theta1 = theta1[1];
							sa->sol[sa->num].theta2 = theta2[1];
							sa->num++;
						}
					}
				}
			}
			break;
		case 1:
			//two theta1, infinite theta2
			sa = (solution_array *)malloc(sizeof(solution_array));
			sa->num = 0;
			sa->sol = NULL;
			// if(lim2->step < 1e-6) {
			// 	return NULL;
			// }
			if(equal(theta1[0], theta1[1])) {
				if(check_limit(theta1[0], lim1)) {
					sa->num = 1;
					sa->sol = (solution *)malloc(sizeof(solution) * sa->num);
					sa->sol[0].theta1 = theta1[0];
					sa->sol[0].theta2 = pre_angle.get_angle(2);
				} else {
					return NULL;
				}
			} else {
				sa->num = 2;
				sa->sol = (solution *)malloc(sizeof(solution) * sa->num);
				int index = 0;
				if(check_limit(theta1[0], lim1)) {
					sa->sol[index].theta1 = theta1[0];
					sa->sol[index].theta2 = pre_angle.get_angle(2);
					index++;
				}
				if(check_limit(theta1[1], lim1)) {
					sa->sol[index].theta1 = theta1[1];
					sa->sol[index].theta2 = pre_angle.get_angle(2);
					index++;
				}
				sa->num = index;
			}
			break;
		case 2:
			//infinite theta1, two theta2
			sa = (solution_array *)malloc(sizeof(solution_array));
			sa->num = 0;
			sa->sol = NULL;
			// if(lim1->step < 1e-6) {
			// 	return NULL;
			// }
			if(equal(theta2[0], theta2[1])) {
				if(check_limit(theta2[0], lim2)) {
					sa->num = 1;
					sa->sol = (solution *)malloc(sizeof(solution) * sa->num);
					sa->sol[0].theta1 = pre_angle.get_angle(1);
					sa->sol[0].theta2 = theta2[0];			
				} else {
					return NULL;
				}
			} else {
				sa->num = 2;
				sa->sol = (solution *)malloc(sizeof(solution) * sa->num);
				int index = 0;
				if(check_limit(theta2[0], lim2)) {
					sa->sol[index].theta1 = pre_angle.get_angle(1);
					sa->sol[index].theta2 = theta2[0];		
					index++;
				}
				if(check_limit(theta2[1], lim2)) {
					sa->sol[index].theta1 = pre_angle.get_angle(1);
					sa->sol[index].theta2 = theta2[1];
					index++;
				}
				sa->num = index;
			}
			break;
		case 3:
			//infinite theta1, infinite theta2
//			return NULL;
			sa = (solution_array *)malloc(sizeof(solution_array));
			sa->num = 1;
			sa->sol = (solution *)malloc(sizeof(solution) * sa->num);
			sa->sol[0].theta1 = pre_angle.get_angle(1);
			sa->sol[0].theta2 = pre_angle.get_angle(2);
			return sa;
		}

		
		if(sa->num > 0) {
			if (sa->num == 1) {
				return sa;
			}
			solution_array *ret = (solution_array *)malloc(sizeof(solution_array));
			ret->num = 1;
			ret->sol = (solution *)malloc(sizeof(solution) * sa->num);
			ret->sol[0] = sa->sol[0];
			double best_diff1 = ret->sol[0].theta1 - pre_angle.get_angle(1);
			double best_diff2 = ret->sol[0].theta2 - pre_angle.get_angle(2);
			double best_dist = best_diff1 * best_diff1 + best_diff2 * best_diff2;
			for(int i = 1; i < sa->num; i++) {
				double tmp1, tmp2;
				tmp1 = sa->sol[i].theta1 - pre_angle.get_angle(1);
				tmp2 = sa->sol[i].theta2 - pre_angle.get_angle(2);
				double length = tmp1 * tmp1 + tmp2 * tmp2;
				if (length < best_dist) {
					best_dist = length;
					ret->sol[0] = sa->sol[i];
				}
			}
			free_solution(sa);			
			return ret;
		}

		free_solution(sa);

		return NULL;
	}
	
	return NULL;
}
