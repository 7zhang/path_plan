#include <math.h>
#include <stdlib.h>
#include "positioner.h"

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
solution_array *positioner_inverse(vector3d *before, vector3d *after, limit *lim1, limit *lim2)
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
			if(sa->num > 0) {
				return sa;
			} else {
				if(sa->sol != NULL) {
					free(sa->sol);
				}

				free(sa);
				return NULL;
			}
		case 1:
			//two theta1, infinite theta2
			sa = (solution_array *)malloc(sizeof(solution_array));
			sa->num = 0;
			sa->sol = NULL;
			if(lim2->step < 1e-6) {
				return NULL;
			}
			if(equal(theta1[0], theta1[1])) {
				if(check_limit(theta1[0], lim1)) {
					sa->num = (lim2->max - lim2->min) / lim2->step;
					sa->sol = (solution *)malloc(sizeof(solution) * sa->num);
					for(i = 0; i < sa->num; ++i) {
						sa->sol[i].theta1 = theta1[0];
						sa->sol[i].theta2 = lim2->min + i * lim2->step;				
					}
				}
			} else {
				int half = (lim2->max - lim2->min) / lim2->step;
				sa->num = 2 * half;
				sa->sol = (solution *)malloc(sizeof(solution) * sa->num);
				int index = 0;
				if(check_limit(theta1[0], lim1)) {
					for(i = 0; i < half; ++i) {
						sa->sol[index].theta1 = theta1[0];
						sa->sol[index].theta2 = lim2->min + i * lim2->step;
						index++;
					}
				}
				if(check_limit(theta1[1], lim1)) {
					for(i = 0; i < half; ++i) {
						sa->sol[index].theta1 = theta1[1];
						sa->sol[index].theta2 = lim2->min + i * lim2->step;
						index++;
					}
				}
				sa->num = index;
			}
			if(sa->num > 0) {
				return sa;
			} else {
				if(sa->sol != NULL) {
					free(sa->sol);
				}

				free(sa);
				return NULL;
			}
		case 2:
			//infinite theta1, two theta2
			sa = (solution_array *)malloc(sizeof(solution_array));
			sa->num = 0;
			sa->sol = NULL;
			if(lim1->step < 1e-6) {
				return NULL;
			}
			if(equal(theta2[0], theta2[1])) {
				if(check_limit(theta2[0], lim2)) {
					sa->num = (lim1->max - lim1->min) / lim1->step;
					sa->sol = (solution *)malloc(sizeof(solution) * sa->num);
					for(j = 0; j < sa->num; ++j) {
						sa->sol[j].theta1 = lim1->min + j * lim1->step;
						sa->sol[j].theta2 = theta2[0];			
					}
				}
			} else {
				int half = (lim1->max - lim1->min) / lim1->step;
				sa->num = 2 * half;
				sa->sol = (solution *)malloc(sizeof(solution) * sa->num);
				int index = 0;
				if(check_limit(theta2[0], lim2)) {
					for(j = 0; j < half; ++j) {
						sa->sol[index].theta1 = lim1->min + j * lim1->step;
						sa->sol[index].theta2 = theta2[0];		
						index++;
					}
				}
				if(check_limit(theta2[1], lim2)) {
					for(j = 0; j < half; ++j) {
						sa->sol[index].theta1 = lim1->min + j * lim1->step;
						sa->sol[index].theta2 = theta2[1];
						index++;
					}
				}
				sa->num = index;
			}
			if(sa->num > 0) {
				return sa;
			} else {
				if(sa->sol != NULL) {
					free(sa->sol);
				}

				free(sa);
				return NULL;
			}
		case 3:
			//infinite theta1, infinite theta2
			return NULL;
		}
	}
	
	return NULL;
}
