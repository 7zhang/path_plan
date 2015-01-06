#include <stdio.h>
#include "triangle_cd.h"
#include "geom.h"
#include "cd_macro.h"
#include "cd.h"

// 0 if no collision
int triangle_cd_para(const triangle *t1, const triangle *t2)
{
	/* double tmptest; */
	/* tmptest = t1->normalvector.x * t1->normalvector.x + t1->normalvector.y * t1->normalvector.y + t1->normalvector.z * t1->normalvector.z; */
	
	/* if (!( tmptest < 1 + MYZERO && tmptest > 1 - MYZERO)) */
	/* 	printf("normal vector error %f\n", tmptest); */

	/* tmptest = t2->normalvector.x * t2->normalvector.x + t2->normalvector.y * t2->normalvector.y + t2->normalvector.z * t2->normalvector.z; */
	
	/* if (!( tmptest < 1 + MYZERO && tmptest > 1 - MYZERO)) */
	/* 	printf("normal vector error %f\n", tmptest); */

	vector3d diff11;
	vector3d diff21;
	vector3d diff31;

	vectorminus(&t2->vertex1, &t1->vertex1, &diff11);
	vectorminus(&t2->vertex2, &t1->vertex1, &diff21);
	vectorminus(&t2->vertex3, &t1->vertex1, &diff31);

	double dot4, dot5, dot6;
	vectordot(&diff11, &t1->normalvector, &dot4);
	vectordot(&diff21, &t1->normalvector, &dot5);
	vectordot(&diff31, &t1->normalvector, &dot6);

	if ((dot4 > POSITIVEZERO && dot5 > POSITIVEZERO && dot6 > POSITIVEZERO) ||
	    (dot4 < NEGATIVEZERO && dot5 < NEGATIVEZERO && dot6 < NEGATIVEZERO)) {
		return 0;
	}
	
	vectorminus(&t1->vertex1, &t2->vertex1, &diff11);
	vectorminus(&t1->vertex2, &t2->vertex1, &diff21);
	vectorminus(&t1->vertex3, &t2->vertex1, &diff31);

	double dot1, dot2, dot3;
	vectordot(&diff11, &t2->normalvector, &dot1);
	vectordot(&diff21, &t2->normalvector, &dot2);
	vectordot(&diff31, &t2->normalvector, &dot3);

	if ((dot1 > POSITIVEZERO && dot2 > POSITIVEZERO && dot3 > POSITIVEZERO) ||
	    (dot1 < NEGATIVEZERO && dot2 < NEGATIVEZERO && dot3 < NEGATIVEZERO)) {
		return 0;
	}
	
	vector3d tangent;//slope of the intersection line
	vector3d p;//one point in the intersection line
	double s1, s2;
	vectordot(&t1->vertex1, &t1->normalvector, &s1);
	vectordot(&t2->vertex1, &t2->normalvector, &s2);
	vectorcross(&t1->normalvector, &t2->normalvector, &tangent);
			
	if (!ISZERO(tangent.z)) {
		p.x = (t2->normalvector.y * s1 - t1->normalvector.y * s2)/tangent.z;
		p.y = (t1->normalvector.x * s2 - t2->normalvector.x * s1)/tangent.z;
		p.z = 0;
	}
	else if (!ISZERO(tangent.y)) {
		p.x = (t1->normalvector.z * s2 - t2->normalvector.z * s1)/tangent.y;
		p.y = 0;
		p.z = (t2->normalvector.x * s1 - t1->normalvector.x * s2)/tangent.y;
	}
	else if (!ISZERO(tangent.x)) {
		p.x = 0;
		p.y = (t2->normalvector.z * s1 - t1->normalvector.z * s2)/tangent.x;
		p.z = (t1->normalvector.y * s2 - t2->normalvector.y * s1)/tangent.x;
	} else
	{
		return 0;
	}

	double para1, para2, para3, para4;
	vector3d tmp;
	if ((dot1 < NEGATIVEZERO && dot2 > POSITIVEZERO && dot3 > POSITIVEZERO) ||
	    (dot1 > POSITIVEZERO && dot2 < NEGATIVEZERO && dot3 < NEGATIVEZERO)) {
		vectorminus(&t1->vertex1, &t1->vertex2, &tmp);
		para1 = common_point(&p, &tangent, &t1->vertex1, &tmp);
		vectorminus(&t1->vertex1, &t1->vertex3, &tmp);
		para2 = common_point(&p, &tangent, &t1->vertex1, &tmp);
	} else if ((dot1 > POSITIVEZERO && dot2 < NEGATIVEZERO && dot3 > POSITIVEZERO) ||
		   (dot1 < NEGATIVEZERO && dot2 > POSITIVEZERO && dot3 < NEGATIVEZERO)) {
		vectorminus(&t1->vertex2, &t1->vertex1, &tmp);
		para1 = common_point(&p, &tangent, &t1->vertex2, &tmp);
		vectorminus(&t1->vertex2, &t1->vertex3, &tmp);
		para2 = common_point(&p, &tangent, &t1->vertex2, &tmp);
	} else if ((dot1 > POSITIVEZERO && dot2 > POSITIVEZERO && dot3 < NEGATIVEZERO) ||
		   (dot1 < NEGATIVEZERO && dot2 < NEGATIVEZERO && dot3 > POSITIVEZERO)) {
		vectorminus(&t1->vertex3, &t1->vertex1, &tmp);
		para1 = common_point(&p, &tangent, &t1->vertex3, &tmp);
		vectorminus(&t1->vertex3, &t1->vertex2, &tmp);
		para2 = common_point(&p, &tangent, &t1->vertex3, &tmp);
	} else {
		return 0;
	}
	
	if ((dot4 < NEGATIVEZERO && dot5 > POSITIVEZERO && dot6 > POSITIVEZERO) ||
	    (dot4 > POSITIVEZERO && dot5 < NEGATIVEZERO && dot6 < NEGATIVEZERO)) {
		vectorminus(&t2->vertex1, &t2->vertex2, &tmp);
		para3 = common_point(&p, &tangent, &t2->vertex1, &tmp);
		vectorminus(&t2->vertex1, &t2->vertex3, &tmp);
		para4 = common_point(&p, &tangent, &t2->vertex1, &tmp);
	} else if ((dot4 > POSITIVEZERO && dot5 < NEGATIVEZERO && dot6 > POSITIVEZERO) ||
		   (dot4 < NEGATIVEZERO && dot5 > POSITIVEZERO && dot6 < NEGATIVEZERO)) {
		vectorminus(&t2->vertex2, &t2->vertex1, &tmp);
		para3 = common_point(&p, &tangent, &t2->vertex2, &tmp);
		vectorminus(&t2->vertex2, &t2->vertex3, &tmp);
		para4 = common_point(&p, &tangent, &t2->vertex2, &tmp);
	} else if ((dot4 > POSITIVEZERO && dot5 > POSITIVEZERO && dot6 < NEGATIVEZERO) ||
		   (dot4 < NEGATIVEZERO && dot5 < NEGATIVEZERO && dot6 > POSITIVEZERO)) {
		vectorminus(&t2->vertex3, &t2->vertex1, &tmp);
		para3 = common_point(&p, &tangent, &t2->vertex3, &tmp);
		vectorminus(&t2->vertex3, &t2->vertex2, &tmp);
		para4 = common_point(&p, &tangent, &t2->vertex3, &tmp);
	} else {
		return 0;
	}	

	double min1, min2, max1, max2;
	min1 = MIN(para1, para2) - MYZERO;
	max1 = MAX(para1, para2) + MYZERO;
	min2 = MIN(para3, para4) - MYZERO;
	max2 = MAX(para3, para4) + MYZERO;

	if (((para1 < max2) && (para1 > min2)) ||
	    ((para2 < max2) && (para2 > min2)) ||
	    ((para3 < max1) && (para3 > min1)) ||
		((para4 < max1) && (para4 > min1)) ) {
#ifdef DEBUG
		printf("\npara1 = %lf, para2 = %lf, para3 = %lf, para4 = %lf\n", para1, para2, para3, para4);
#endif
				return 1;
	}
	
	return 0;
}


// 0 if no collision
int triangle_cd(const triangle *t1, const triangle *t2)
{
	/* double tmptest; */
	/* tmptest = t1->normalvector.x * t1->normalvector.x + t1->normalvector.y * t1->normalvector.y + t1->normalvector.z * t1->normalvector.z; */
	
	/* if (!( tmptest < 1 + MYZERO && tmptest > 1 - MYZERO)) */
	/* 	printf("normal vector error %f\n", tmptest); */

	/* tmptest = t2->normalvector.x * t2->normalvector.x + t2->normalvector.y * t2->normalvector.y + t2->normalvector.z * t2->normalvector.z; */
	
	/* if (!( tmptest < 1 + MYZERO && tmptest > 1 - MYZERO)) */
	/* 	printf("normal vector error %f\n", tmptest); */

	vector3d diff11;
	vector3d diff21;
	vector3d diff31;

	vectorminus(&t2->vertex1, &t1->vertex1, &diff11);
	vectorminus(&t2->vertex2, &t1->vertex1, &diff21);
	vectorminus(&t2->vertex3, &t1->vertex1, &diff31);

	double dot4, dot5, dot6;
	vectordot(&diff11, &t1->normalvector, &dot4);
	vectordot(&diff21, &t1->normalvector, &dot5);
	vectordot(&diff31, &t1->normalvector, &dot6);

	if ((dot4 > POSITIVEZERO && dot5 > POSITIVEZERO && dot6 > POSITIVEZERO) ||
	    (dot4 < NEGATIVEZERO && dot5 < NEGATIVEZERO && dot6 < NEGATIVEZERO)) {
		return 0;
	}
	
	vectorminus(&t1->vertex1, &t2->vertex1, &diff11);
	vectorminus(&t1->vertex2, &t2->vertex1, &diff21);
	vectorminus(&t1->vertex3, &t2->vertex1, &diff31);

	double dot1, dot2, dot3;
	vectordot(&diff11, &t2->normalvector, &dot1);
	vectordot(&diff21, &t2->normalvector, &dot2);
	vectordot(&diff31, &t2->normalvector, &dot3);

	if ((dot1 > POSITIVEZERO && dot2 > POSITIVEZERO && dot3 > POSITIVEZERO) ||
	    (dot1 < NEGATIVEZERO && dot2 < NEGATIVEZERO && dot3 < NEGATIVEZERO)) {
		return 0;
	}
	

	vector3d cross1, cross2, cross3, cross4;
	vector3d tangent;
	if ((dot1 < NEGATIVEZERO && dot2 >= POSITIVEZERO && dot3 >= POSITIVEZERO) ||
	    (dot1 > POSITIVEZERO && dot2 <= NEGATIVEZERO && dot3 <= NEGATIVEZERO)) {		
		vectorminus(&t1->vertex1, &t1->vertex2, &tangent);
		intersect_line_surface(&t1->vertex1, &tangent, &t2->vertex1, &t2->normalvector, &cross1);
		vectorminus(&t1->vertex1, &t1->vertex3, &tangent);
		intersect_line_surface(&t1->vertex1, &tangent, &t2->vertex1, &t2->normalvector, &cross2);
	} else if ((dot1 >= POSITIVEZERO && dot2 < NEGATIVEZERO && dot3 >= POSITIVEZERO) ||
		   (dot1 <= NEGATIVEZERO && dot2 > POSITIVEZERO && dot3 <= NEGATIVEZERO)) {
		vectorminus(&t1->vertex2, &t1->vertex1, &tangent);
		intersect_line_surface(&t1->vertex2, &tangent, &t2->vertex1, &t2->normalvector, &cross1);
		vectorminus(&t1->vertex2, &t1->vertex3, &tangent);
		intersect_line_surface(&t1->vertex2, &tangent, &t2->vertex1, &t2->normalvector, &cross2);
	} else if ((dot1 >= POSITIVEZERO && dot2 >= POSITIVEZERO && dot3 < NEGATIVEZERO) ||
		   (dot1 <= NEGATIVEZERO && dot2 <= NEGATIVEZERO && dot3 > POSITIVEZERO)) {
		vectorminus(&t1->vertex3, &t1->vertex1, &tangent);
		intersect_line_surface(&t1->vertex3, &tangent, &t2->vertex1, &t2->normalvector, &cross1);
		vectorminus(&t1->vertex3, &t1->vertex2, &tangent);
		intersect_line_surface(&t1->vertex3, &tangent, &t2->vertex1, &t2->normalvector, &cross2);
	} else {
		return 0;
	}
	
	if ((dot4 < NEGATIVEZERO && dot5 >= POSITIVEZERO && dot6 >= POSITIVEZERO) ||
	    (dot4 > POSITIVEZERO && dot5 <= NEGATIVEZERO && dot6 <= NEGATIVEZERO)) {
		vectorminus(&t2->vertex1, &t2->vertex2, &tangent);
		intersect_line_surface(&t2->vertex1, &tangent, &t1->vertex1, &t1->normalvector, &cross3);
		vectorminus(&t2->vertex1, &t2->vertex3, &tangent);
		intersect_line_surface(&t2->vertex1, &tangent, &t1->vertex1, &t1->normalvector, &cross4);
	} else if ((dot4 >= POSITIVEZERO && dot5 < NEGATIVEZERO && dot6 >= POSITIVEZERO) ||
		   (dot4 <= NEGATIVEZERO && dot5 > POSITIVEZERO && dot6 <= NEGATIVEZERO)) {
		vectorminus(&t2->vertex2, &t2->vertex1, &tangent);
		intersect_line_surface(&t2->vertex2, &tangent, &t1->vertex1, &t1->normalvector, &cross3);
		vectorminus(&t2->vertex2, &t2->vertex3, &tangent);
		intersect_line_surface(&t2->vertex2, &tangent, &t1->vertex1, &t1->normalvector, &cross4);
	} else if ((dot4 >= POSITIVEZERO && dot5 >= POSITIVEZERO && dot6 < NEGATIVEZERO) ||
		   (dot4 <= NEGATIVEZERO && dot5 <= NEGATIVEZERO && dot6 > POSITIVEZERO)) {
		vectorminus(&t2->vertex3, &t2->vertex1, &tangent);
		intersect_line_surface(&t2->vertex3, &tangent, &t1->vertex1, &t1->normalvector, &cross3);
		vectorminus(&t2->vertex3, &t2->vertex2, &tangent);
		intersect_line_surface(&t2->vertex3, &tangent, &t1->vertex1, &t1->normalvector, &cross4);
	} else {
		return 0;
	}
	
	double para1, para2, para3, para4;
	para1 = 0.0;
	para2 = 1.0;
	double dist1, dist2, dist3, dist4;
	vector3d diff1, diff2, diff3, diff4;
	vectorminus(&cross2, &cross1, &diff1);
	vectorminus(&cross4, &cross3, &diff2);
	length(&diff1, &dist1);
	length(&diff2, &dist2);

	int flag = 0;
	if(dist1 > dist2) {
		vectorminus(&cross3, &cross1, &diff3);
		vectorminus(&cross4, &cross1, &diff4);
		length(&diff3, &dist3);
		length(&diff4, &dist4);

		para3 = dist3 / dist1;
		para4 = dist4 / dist1;
		add_sign(&diff3, &diff1, &para3);
		add_sign(&diff4, &diff1, &para4);
	} else {
		vectorminus(&cross1, &cross3, &diff3);
		vectorminus(&cross2, &cross3, &diff4);
		length(&diff3, &dist3);
		length(&diff4, &dist4);
		
		para3 = dist3 / dist2;
		para4 = dist4 / dist2;
		add_sign(&diff3, &diff2, &para3);
		add_sign(&diff4, &diff2, &para4);
		flag = 1;
	}


	double min1, min2, max1, max2;
	min1 = MIN(para1, para2) - MYZERO;
	max1 = MAX(para1, para2) + MYZERO;
	min2 = MIN(para3, para4) - MYZERO;
	max2 = MAX(para3, para4) + MYZERO;

	if (para1 < max2 && para1 > min2 ||
	    para2 < max2 && para2 > min2 ||
	    para3 < max1 && para3 > min1 ||
		para4 < max1 && para4 > min1) {		
#ifdef DEBUG
		show_point(&cross1);
		show_point(&cross2);
		show_point(&cross3);
		show_point(&cross4);
		printf("\npara1 = %lf, para2 = %lf, para3 = %lf, para4 = %lf\n", para1, para2, para3, para4);
#endif
				return 1;
	}
	
	return 0;
}















