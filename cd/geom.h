#ifndef _GEOM_H_
#define _GEOM_H_

#ifdef DEBUG
#include <stdio.h>
#endif

#include <math.h>
#include "cd.h"
#include "cd_macro.h"
			
static inline void vectoradd(const vector3d *v1, const vector3d *v2, vector3d *sum)
{
	sum->x = v1->x + v2->x;
	sum->y = v1->y + v2->y;
	sum->z = v1->z + v2->z;
}

static inline void vectorminus(const vector3d *lhs, const vector3d *rhs, vector3d *diff)
{
	diff->x = lhs->x - rhs->x;
	diff->y = lhs->y - rhs->y;
	diff->z = lhs->z - rhs->z;
}

static inline void vectordot(const vector3d *v1, const vector3d *v2, double *dot)
{
	*dot = v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

static inline void length(const vector3d *v, double *length)
{
	double tmp;
	vectordot(v, v, &tmp);
	*length = sqrt(tmp);
}

static inline void vectorcross(const vector3d *lhs, const vector3d *rhs, vector3d *cross)
{
	cross->x = lhs->y * rhs->z - lhs->z * rhs->y;
	cross->y = lhs->z * rhs->x - lhs->x * rhs->z;
	cross->z = lhs->x * rhs->y - lhs->y * rhs->x;
}

static inline double distance(const vector3d *a, const vector3d *b)
{
	return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y)
		    + (a->z - b->z) * (a->z - b->z)); 
}

static inline double common_point(const vector3d *p1, const vector3d *tangent1,
				  const vector3d *p2, const vector3d *tangent2)
{
	vector3d diff;
	vectorminus(p2, p1, &diff);

	vector3d lhs, rhs;
	
	vectorcross(tangent1, tangent2, &rhs);
	vectorcross(&diff, tangent2, &lhs);
	
	double tmp1, tmp2;
	
	vectordot(&rhs, &rhs, &tmp1);
	vectordot(&lhs, &lhs, &tmp2);

	if (fabs(rhs.x) > POSITIVEZERO) {
		if ((lhs.x > 0 && rhs.x > 0) || (lhs.x < 0 && rhs.x < 0)) {
			return  sqrt(tmp2/tmp1);		
		} else {
			return -sqrt(tmp2/tmp1);
		}
	}

	if (fabs(rhs.y) > POSITIVEZERO) {
		if ((lhs.y > 0 && rhs.y > 0) || (lhs.y < 0 && rhs.y < 0)) {
			return  sqrt(tmp2/tmp1);		
		} else {
			return -sqrt(tmp2/tmp1);
		}
	}

	if (fabs(rhs.z) > POSITIVEZERO) {
		if ((lhs.z > 0 && rhs.z > 0) || (lhs.z < 0 && rhs.z < 0)) {
			return  sqrt(tmp2/tmp1);		
		} else {
			return -sqrt(tmp2/tmp1);
		}
	}
}

//lhs = t * rhs
static inline void add_sign(const vector3d *lhs, const vector3d *rhs, double *t)
{
	double tmp;
	tmp = *t;
	if (fabs(rhs->x) > POSITIVEZERO) {
		if ((lhs->x > 0 && rhs->x > 0) || (lhs->x < 0 && rhs->x < 0)) {
			return ;		
		} else {
			*t = -tmp;
			return ;
		}
	}
	
	if (fabs(rhs->y) > POSITIVEZERO) {
		if ((lhs->y > 0 && rhs->y > 0) || (lhs->y < 0 && rhs->y < 0)) {
			return ;		
		} else {
			*t = -tmp;
			return ;
		}
	}
	
	if (fabs(rhs->z) > POSITIVEZERO) {
		if ((lhs->z > 0 && rhs->z > 0) || (lhs->z < 0 && rhs->z < 0)) {
			return ;		
		} else {
			*t = -tmp;
			return ;
		}
	}
}

static inline void intersect_line_surface(const vector3d *line_point, const vector3d *line_tangent
										  , const vector3d *surface_point, const vector3d *surface_normal
										  , vector3d *cross)
{
	vector3d diff;
	vectorminus(surface_point, line_point, &diff);
	double tmp1, tmp2;
	vectordot(line_tangent, surface_normal, &tmp1);
	vectordot(&diff, surface_normal, &tmp2);
	
#ifdef DEBUG
	if(fabs(tmp1) < POSITIVEZERO) {
		printf("Error, possibly divided by zero\n");
	}
#endif
	
	double t = tmp2 / tmp1;
	cross->x = line_point->x + t * line_tangent->x;
	cross->y = line_point->y + t * line_tangent->y;
	cross->z = line_point->z + t * line_tangent->z;
}

static inline void point_transform(const matrix m, const vector3d *vec, const vector3d *from, vector3d *to)
{
	to->x = m[0][0] * from->x
		+ m[0][1] * from->y + m[0][2] * from->z + vec->x;
	to->y = m[1][0] * from->x
		+ m[1][1] * from->y + m[1][2] * from->z + vec->y;
	to->z = m[2][0] * from->x
		+ m[2][1] * from->y + m[2][2] * from->z + vec->z;
}

static inline void triangle_transform(const matrix m, const vector3d *vec, const triangle *from, triangle *to)
{
	vector3d tmp = {0.0, 0.0, 0.0};

	point_transform(m, &tmp, &from->normalvector, &to->normalvector);
	point_transform(m, vec, &from->vertex1, &to->vertex1);
	point_transform(m, vec, &from->vertex2, &to->vertex2);
	point_transform(m, vec, &from->vertex3, &to->vertex3);
}

static inline void swap(double *a, double *b)
{
	double tmp;
	tmp = *a;
	*a = *b;
	*b = tmp;
}

static inline void transpose(matrix m)
{
	swap(&m[0][1], &m[1][0]);
	swap(&m[0][2], &m[2][0]);
	swap(&m[1][2], &m[2][1]);
}

static inline void matrix_multiply(const matrix left, const matrix right, matrix ret)
{
	ret[0][0] = left[0][0] * right[0][0] + left[0][1] * right[1][0] + left[0][2] * right[2][0];
	ret[0][1] = left[0][0] * right[0][1] + left[0][1] * right[1][1] + left[0][2] * right[2][1];
	ret[0][2] = left[0][0] * right[0][2] + left[0][1] * right[1][2] + left[0][2] * right[2][2];

	ret[1][0] = left[1][0] * right[0][0] + left[1][1] * right[1][0] + left[1][2] * right[2][0];
	ret[1][1] = left[1][0] * right[0][1] + left[1][1] * right[1][1] + left[1][2] * right[2][1];
	ret[1][2] = left[1][0] * right[0][2] + left[1][1] * right[1][2] + left[1][2] * right[2][2];

	ret[2][0] = left[2][0] * right[0][0] + left[2][1] * right[1][0] + left[2][2] * right[2][0];
	ret[2][1] = left[2][0] * right[0][1] + left[2][1] * right[1][1] + left[2][2] * right[2][1];
	ret[2][2] = left[2][0] * right[0][2] + left[2][1] * right[1][2] + left[2][2] * right[2][2];
}

#ifdef DEBUG
static void show_point(vector3d *p)
{
	printf("point: x = %lf, y = %lf, z = %lf\n", p->x, p->y, p->z);
}
#endif

#endif /* _GEOM_H_ */

















