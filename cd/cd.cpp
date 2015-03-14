#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cd.h"
#include "triangle_cd.h"
#include "loadstl.h"
#include "geom.h"
#include "cd_macro.h"

#ifdef DEBUG
int volumecount;
int cdcount;
int last_count;
int triangle_cd_count;
#endif

int triangle_allocation(const volumenode *parent, volumenode *left, volumenode *right);

//return 0 if no collision
static inline int volumecd(const volume *v1, const volume *v2)
{
	if (v1->xmax < v2->xmin || v1->xmin > v2->xmax)
		return 0;
	
	if (v1->ymax < v2->ymin || v1->ymin > v2->ymax)
		return 0;
	
	if (v1->zmax < v2->zmin || v1->zmin > v2->zmax)
		return 0;
	
	return 1;
}

void vertex_copy(const vector3d *from, vector3d *to)
{
	to->x = from->x;
	to->y = from->y;
	to->z = from->z;
}

void middle(const vector3d *from1, const vector3d *from2, vector3d *to)
{
	to->x = (from1->x + from2->x) / 2;
	to->y = (from1->y + from2->y) / 2;
	to->z = (from1->z + from2->z) / 2;
}

int triangle_cut_recur(triangle **array, int *n, int *index, triangle *t, int maxlength)
{
	double d1, d2, d3;
	d1 = distance(&t->vertex1, &t->vertex2);
	d2 = distance(&t->vertex1, &t->vertex3);
	d3 = distance(&t->vertex2, &t->vertex3);
	
	triangle tmp1, tmp2;
	vector3d mid;

	if (d1 > maxlength && d1 >= d2 && d1 >= d3) {
		middle(&t->vertex1, &t->vertex2, &mid);
		
		vertex_copy(&t->normalvector, &tmp1.normalvector);		
		vertex_copy(&t->vertex1, &tmp1.vertex1);
		vertex_copy(&t->vertex3, &tmp1.vertex2);
		vertex_copy(&mid, &tmp1.vertex3);
		tmp1.parent = t->parent;

		vertex_copy(&t->normalvector, &tmp2.normalvector);		
		vertex_copy(&t->vertex2, &tmp2.vertex1);
		vertex_copy(&t->vertex3, &tmp2.vertex2);
		vertex_copy(&mid, &tmp2.vertex3);
		tmp2.parent = t->parent;
		
		if(triangle_cut_recur(array, n, index, &tmp1, maxlength) == 1) {
			return 1;
		}
		if(triangle_cut_recur(array, n, index, &tmp2, maxlength) == 1) {
			return 1;
		}
	} else if (d2 > maxlength && d2 >= d1 && d2 >= d3) {
		middle(&t->vertex1, &t->vertex3, &mid);
		
		vertex_copy(&t->normalvector, &tmp1.normalvector);		
		vertex_copy(&t->vertex1, &tmp1.vertex1);
		vertex_copy(&t->vertex2, &tmp1.vertex2);
		vertex_copy(&mid, &tmp1.vertex3);
		tmp1.parent = t->parent;

		vertex_copy(&t->normalvector, &tmp2.normalvector);		
		vertex_copy(&t->vertex2, &tmp2.vertex1);
		vertex_copy(&t->vertex3, &tmp2.vertex2);
		vertex_copy(&mid, &tmp2.vertex3);
		tmp2.parent = t->parent;

		if(triangle_cut_recur(array, n, index, &tmp1, maxlength) == 1) {
			return 1;
		}
		if(triangle_cut_recur(array, n, index, &tmp2, maxlength) == 1) {
			return 1;
		}
	} else if (d3 > maxlength && d3 >= d1 && d3 >= d2) {
		middle(&t->vertex2, &t->vertex3, &mid);
		
		vertex_copy(&t->normalvector, &tmp1.normalvector);		
		vertex_copy(&t->vertex1, &tmp1.vertex1);
		vertex_copy(&t->vertex2, &tmp1.vertex2);
		vertex_copy(&mid, &tmp1.vertex3);
		tmp1.parent = t->parent;

		vertex_copy(&t->normalvector, &tmp2.normalvector);		
		vertex_copy(&t->vertex1, &tmp2.vertex1);
		vertex_copy(&t->vertex3, &tmp2.vertex2);
		vertex_copy(&mid, &tmp2.vertex3);
		tmp2.parent = t->parent;
		
		if(triangle_cut_recur(array, n, index, &tmp1, maxlength) == 1) {
			return 1;
		}
		if(triangle_cut_recur(array, n, index, &tmp2, maxlength) == 1) {
			return 1;
		}
	} else {
		if (*index >= *n) {
// 			if(*n > 1500000) {
// #ifdef DEBUG
// 				printf("Error:triangle number reaches upper limit\n");
// #endif
// 				return 1;
// 			}
			
			int triangle_num_new;
			if(*n < 500000) {
				triangle_num_new = 2 * (*n);			
			} else {
				triangle_num_new = 1.1 * (*n);				
			}

			if(triangle_num_new <= *n) {
#ifdef DEBUG
				printf("Error:triangle_num_new <= *n\n");
#endif
				return 1;
			}
			triangle *tt;
			tt = (triangle *)malloc(sizeof(triangle) * triangle_num_new);
			if(tt == NULL) {
#ifdef DEBUG
				printf("Error:malloc returns null\n");
#endif
				return 1;
			}

			memcpy(tt, *array, sizeof(triangle) * (*n));
			*n = triangle_num_new;
			free(*array);
			*array = tt;			
		}

		vertex_copy(&t->normalvector, &((*array)[*index]).normalvector);
		vertex_copy(&t->vertex1, &((*array)[*index]).vertex1);
		vertex_copy(&t->vertex2, &((*array)[*index]).vertex2);
		vertex_copy(&t->vertex3, &((*array)[*index]).vertex3);
		((*array)[*index]).parent = t->parent;
		
		(*index)++;
	}
		
	return 0;
}

int triangle_cut(stldata *stl, int maxlength)
{
	int *n = (int *)malloc(sizeof(int));
	int *index = (int *)malloc(sizeof(int));
	triangle *tmp = (triangle *)malloc(sizeof(triangle) * 2 * stl->num);
	triangle **array = (triangle **)malloc(sizeof(triangle *));
	*n = stl->num;
	*index = 0;
	*array = tmp;
	
	int i;
	for (i = 0; i < stl->num; ++i)
	{
		if(triangle_cut_recur(array, n, index, &stl->ptriangle[i], maxlength) == 1) {
#ifdef DEBUG
			printf("Error:triangle_cut_recur\n");
#endif
			return 1;
		}
	}
	
	stl->num_cutted = *index;
//	free(stl->ptriangle);

	stl->ptriangle_cutted = (triangle *)malloc(sizeof(triangle) * (*index));
	if(stl->ptriangle_cutted == NULL) {
#ifdef DEBUG
		printf("Error:malloc returns null:ptriangle_cutted\n");
#endif
		return 1;
	}

	memcpy(stl->ptriangle_cutted, *array, sizeof(triangle) * (*index));
	free(*array);	
	free(n);
	free(index);
	free(array);

	return 0;
}

/* use the tarray and tindex to build the volume v
 */
int build_volume(volumenode *vnode)
{
	int i;
	triangle *ttmp = &vnode->tarry[vnode->tindex[0]];
	volume *vtmp = &vnode->v;
	vtmp->xmin = ttmp->vertex1.x;
	vtmp->xmax = ttmp->vertex1.x;
	vtmp->ymin = ttmp->vertex1.y;
	vtmp->ymax = ttmp->vertex1.y;
	vtmp->zmin = ttmp->vertex1.z;
	vtmp->zmax = ttmp->vertex1.z;

#ifdef DEBUG
	if (vnode->trianglenum == 0) {
		printf("buildvolume error! vnode->trianglenum = 0\n");
		return 1;
	}
#endif

	for (i = 0; i < vnode->trianglenum; ++i) {

#ifdef DEBUG
		if (vnode->tindex[i] >= vnode->tarraysize) {
			printf("index error! vnode->tindex[%d] = %d, while vnode->trianglenum = %d\n",
			       i, vnode->tindex[i], vnode->trianglenum);
			return 1;
		}
#endif
		  
		ttmp = &vnode->tarry[vnode->tindex[i]];

		if (ttmp->vertex1.x < vtmp->xmin)
			vtmp->xmin = ttmp->vertex1.x;
		if (ttmp->vertex1.x > vtmp->xmax)
			vtmp->xmax = ttmp->vertex1.x;
		if (ttmp->vertex1.y < vtmp->ymin)
			vtmp->ymin = ttmp->vertex1.y;
		if (ttmp->vertex1.y > vtmp->ymax)
			vtmp->ymax = ttmp->vertex1.y;
		if (ttmp->vertex1.z < vtmp->zmin)
			vtmp->zmin = ttmp->vertex1.z;
		if (ttmp->vertex1.z > vtmp->zmax)
			vtmp->zmax = ttmp->vertex1.z;

		if (ttmp->vertex2.x < vtmp->xmin)
			vtmp->xmin = ttmp->vertex2.x;
		if (ttmp->vertex2.x > vtmp->xmax)
			vtmp->xmax = ttmp->vertex2.x;
		if (ttmp->vertex2.y < vtmp->ymin)
			vtmp->ymin = ttmp->vertex2.y;
		if (ttmp->vertex2.y > vtmp->ymax)
			vtmp->ymax = ttmp->vertex2.y;
		if (ttmp->vertex2.z < vtmp->zmin)
			vtmp->zmin = ttmp->vertex2.z;
		if (ttmp->vertex2.z > vtmp->zmax)
			vtmp->zmax = ttmp->vertex2.z;
	
		if (ttmp->vertex3.x < vtmp->xmin)
			vtmp->xmin = ttmp->vertex3.x;
		if (ttmp->vertex3.x > vtmp->xmax)
			vtmp->xmax = ttmp->vertex3.x;
		if (ttmp->vertex3.y < vtmp->ymin)
			vtmp->ymin = ttmp->vertex3.y;
		if (ttmp->vertex3.y > vtmp->ymax)
			vtmp->ymax = ttmp->vertex3.y;
		if (ttmp->vertex3.z < vtmp->zmin)
			vtmp->zmin = ttmp->vertex3.z;
		if (ttmp->vertex3.z > vtmp->zmax)
			vtmp->zmax = ttmp->vertex3.z;
	}

#ifdef DEBUG
	if (vnode->v.xmax - vnode->v.xmin < POSITIVEZERO &&
	    vnode->v.ymax - vnode->v.ymin < POSITIVEZERO &&
	    vnode->v.zmax - vnode->v.zmin < POSITIVEZERO) {
		printf("error:x y z length all equal 0\n");
	}
#endif

#ifdef WARNING
	if (vnode->v.ymax - vnode->v.ymin < POSITIVEZERO) {
		printf("warning:y length equal 0\n");
	}
	
	if (vnode->v.zmax - vnode->v.zmin < POSITIVEZERO) {
		printf("warning:z length equal 0\n");
	}
#endif
	
	return 0;
}

void update_volume(const matrix m, const vector3d *v, const volume *from, volume *to)
{
	vector3d updated_vertex[8];
	
	vector3d tmp1 = {from->xmin, from->ymin, from->zmin};
	vector3d tmp2 = {from->xmax, from->ymin, from->zmin};
	vector3d tmp3 = {from->xmin, from->ymax, from->zmin};
	vector3d tmp4 = {from->xmin, from->ymin, from->zmax};
	
	point_transform(m, v, &tmp1, &updated_vertex[0]);
	point_transform(m, v, &tmp2, &updated_vertex[1]);
	point_transform(m, v, &tmp3, &updated_vertex[2]);
	point_transform(m, v, &tmp4, &updated_vertex[3]);

	vector3d buffer1, buffer2, buffer3;
	
	vectorminus(&updated_vertex[1], &updated_vertex[0], &buffer1);
	vectorminus(&updated_vertex[2], &updated_vertex[0], &buffer2);
	vectorminus(&updated_vertex[3], &updated_vertex[0], &buffer3);

	vectoradd(&buffer1, &updated_vertex[3], &updated_vertex[4]);
	vectoradd(&buffer2, &updated_vertex[1], &updated_vertex[5]);
	vectoradd(&buffer2, &updated_vertex[3], &updated_vertex[6]);
	vectoradd(&buffer3, &updated_vertex[5], &updated_vertex[7]);

	int i;
	vector3d *tmp;

	to->xmin = updated_vertex[0].x;
	to->xmax = updated_vertex[0].x;
	to->ymin = updated_vertex[0].y;
	to->ymax = updated_vertex[0].y;
	to->zmin = updated_vertex[0].z;
	to->zmax = updated_vertex[0].z;

	for (i = 0; i < 8; ++i) {
		tmp = &updated_vertex[i];

		if (tmp->x < to->xmin)
			to->xmin = tmp->x;
		if (tmp->x > to->xmax)
			to->xmax = tmp->x;
		if (tmp->y < to->ymin)
			to->ymin = tmp->y;
		if (tmp->y > to->ymax)
			to->ymax = tmp->y;
		if (tmp->z < to->zmin)
			to->zmin = tmp->z;
		if (tmp->z > to->zmax)
			to->zmax = tmp->z;
	}
}

/* return the depth if success, -1 if error */
int build_tree_recur(volumenode *vnode, int maxtriangle)
{
	volumenode *left = (volumenode *)malloc(sizeof(volumenode));
	volumenode *right = (volumenode *)malloc(sizeof(volumenode));
	
#ifdef DEBUG
	if (left == NULL || right == NULL) {
		printf("build_tree_recur malloc error!\n");
		return -1;
	}
#endif
	
	memset(left, 0, sizeof(volumenode));
	memset(right, 0, sizeof(volumenode));

	left->tarry = vnode->tarry;
	right->tarry = vnode->tarry;
	left->tarraysize = vnode->tarraysize;
	right->tarraysize = vnode->tarraysize;

	int tmp = triangle_allocation(vnode, left, right);

	switch(tmp){
	case 0:
		if (build_volume(left) != 0) {
			printf("buildvolume(left) error!\n");
			free(left->tindex);
			free(right->tindex);
			free(left);
			free(right);
			
			return -1;
		}
			      
		if (build_volume(right) != 0) {
			printf("buildvolume(right) error!\n");
			free(left->tindex);
			free(right->tindex);
			free(left);
			free(right);

			return -1;
		}
		
		vnode->last = 0;

		left->parent = vnode;
		right->parent = vnode;
		
		vnode->child1 = left;
		vnode->child2 = right;
//		left->m = vnode->m;
//		left->vector = vnode->vector;
//		right->m = vnode->m;
//		right->vector = vnode->vector;
		
		free(vnode->tindex);
		/* vnode->trianglenum = 0; */
#ifdef DEBUG
		volumecount += 2;
#endif
		if (left->trianglenum > maxtriangle) {
			if (build_tree_recur(left, maxtriangle) == -1) {
				printf("build_tree_recur(left) error! left->trianglenum = %d\n",
				       left->trianglenum);
				return -1;
			}
		} else {
			left->last = 1;
#ifdef DEBUG
			last_count++;
#endif
		}

		if (right->trianglenum > maxtriangle) {
			if (build_tree_recur(right, maxtriangle) == -1) {
				printf("build_tree_recur(right) error! right->trianglenum = %d\n",
				       right->trianglenum);
				return -1;
			}
		} else {
			right->last = 1;
#ifdef DEBUG
			last_count++;
#endif
		}

		return 0;
	case 1:
		free(left);
		free(right);
		vnode->last = 1;
#ifdef DEBUG
		last_count++;
#endif
		vnode->child1 = NULL;
		vnode->child2 = NULL;

		return 0;		
	case 2:
		free(left);
		free(right);
		printf("triangleallocation error!\n");
		
		return -1;
	}
}

int triangle_allocation(const volumenode *parent, volumenode *left, volumenode *right)
{
	/* if (parent->trianglenum < 1) { */
	/* 	printf("parent->trianglenum = %d\n", parent->trianglenum); */
	/* 	return 1; */
	/* } */

	int *tmp1, *tmp2;
	tmp1 = (int *)malloc(sizeof(int) * parent->trianglenum);

	if (tmp1 == NULL){
		printf("triangleallocation malloc error!\n");
		
		left->tindex = NULL;
		left->trianglenum = 0;
		right->tindex = NULL;
		right->trianglenum = 0;

		return 2;
	}

	tmp2 = (int *)malloc(sizeof(int) * parent->trianglenum);

	if (tmp2 == NULL){
		printf("triangleallocation malloc error!\n");
		
		free(tmp1);
		left->tindex = NULL;
		left->trianglenum = 0;
		right->tindex = NULL;
		right->trianglenum = 0;

		return 2;
	}

	memset(tmp1, 0, sizeof(int) * parent->trianglenum);
	memset(tmp2, 0, sizeof(int) * parent->trianglenum);

	//try x axis
	double len1, len2, len3;
	
	len1 = parent->v.xmax - parent->v.xmin;
	len2 = parent->v.ymax - parent->v.ymin;
	len3 = parent->v.zmax - parent->v.zmin;

#ifdef DEBUG
	if (len1 < MYZERO && len2 < MYZERO && len3 < MYZERO)
	{
		/* free(tmp1); */
		/* free(tmp2); */
		/* left->tindex = NULL; */
		/* left->trianglenum = 0; */
		/* right->tindex = NULL; */
		/* right->trianglenum = 0; */
	
		printf("fatal error!\n");
	}
#endif

	int seq[3];
	if (len1 > len2 && len2 > len3) {
		seq[0] = 1;
		seq[1] = 2;
		seq[2] = 3;
	} else if (len1 > len3 && len3 > len2) {
		seq[0] = 1;
		seq[1] = 3;
		seq[2] = 2;
	} else if (len2 > len1 && len1 > len3) {
		seq[0] = 2;
		seq[1] = 1;
		seq[2] = 3;
	} else if (len2 > len3 && len3 > len1) {
		seq[0] = 2;
		seq[1] = 3;
		seq[2] = 1;
	} else if (len3 > len1 && len1 > len2) {
		seq[0] = 3;
		seq[1] = 1;
		seq[2] = 2;
	} else if (len3 > len2 && len2 > len1) {
		seq[0] = 3;
		seq[1] = 2;
		seq[2] = 1;
	} else {
		seq[0] = 1;
		seq[1] = 2;
		seq[2] = 3;
	}
	
	int i, j;
	int leftcount = 0;
	int rightcount = 0;
	int success = 0;
	double middlex, middley, middlez, centerx, centery, centerz;
		
	for (i = 0; i < 3; ++i) {
		leftcount = 0;
		rightcount = 0;

		switch(seq[i]) {
		case 1:
			middlex = (parent->v.xmin + parent->v.xmax)/2.0;
		
			for (j = 0; j < parent->trianglenum; ++j) {
				centerx = (parent->tarry[parent->tindex[j]].vertex1.x
					   + parent->tarry[parent->tindex[j]].vertex2.x
					   + parent->tarry[parent->tindex[j]].vertex3.x)/3.0;

				/* if (parent->tindex[j] == 897) */
				/* 	printf("897\n"); */
				
				if (centerx < middlex) {
					leftcount++;
					tmp1[leftcount-1] = parent->tindex[j];
				} else {
					rightcount++;
					tmp2[rightcount-1] = parent->tindex[j];
				}
			}
			
			if (leftcount > 0 && rightcount > 0) {
				success = 1;
			}
			
			break;
		case 2:
			middley = (parent->v.ymin + parent->v.ymax)/2.0;
		
			for (j = 0; j < parent->trianglenum; ++j) {
				centery = (parent->tarry[parent->tindex[j]].vertex1.y
					   + parent->tarry[parent->tindex[j]].vertex2.y
					   + parent->tarry[parent->tindex[j]].vertex3.y)/3.0;
				
				/* if (parent->tindex[j] == 897) */
				/* 	printf("897\n"); */
				
				if (centery < middley) {
					leftcount++;
					tmp1[leftcount-1] = parent->tindex[j];
				} else {
					rightcount++;
					tmp2[rightcount-1] = parent->tindex[j];
				}
			}
			
			if (leftcount > 0 && rightcount > 0) {
				success = 1;
			}
			
			break;
		case 3:
			middlez = (parent->v.zmin + parent->v.zmax)/2.0;
		
			for (j = 0; j < parent->trianglenum; ++j) {
				centerz = (parent->tarry[parent->tindex[j]].vertex1.z
					   + parent->tarry[parent->tindex[j]].vertex2.z
					   + parent->tarry[parent->tindex[j]].vertex3.z)/3.0;

				/* if (parent->tindex[j] == 897) */
				/* 	printf("897\n"); */
				
				if (centerz < middlez) {
					leftcount++;
					tmp1[leftcount-1] = parent->tindex[j];
				} else {
					rightcount++;
					tmp2[rightcount-1] = parent->tindex[j];
				}
			}
			
			if (leftcount > 0 && rightcount > 0)
				success = 1;

			break;
		default:
			success = 0;

#ifdef WARNING
			printf("warning: triangleallocation switch reach default\n!");
#endif

			break;
		}

		if (success == 1) {
			/* printf("i = %d\n", i);		 */
			break;
		}
	}

	if (success == 0) {
		/* printf("can't split!\n"); */
		/* printf("parent->trianglenum = %d\n", parent->trianglenum); */
		/* printf("leftcount = %d, tmp1[0] = %d, tmp1[leftcount-1] = %d\n", */
		/*        leftcount, tmp1[0], tmp1[leftcount-1]); */
		/* printf("rightcount = %d, tmp2[0] = %d, tmp2[rightcount-1] = %d\n", */
		/*        rightcount, tmp2[0], tmp2[rightcount-1]); */

		/* if (leftcount > 0 || rightcount > 0) { */
		/* 	printf("len1 = %lf, len2 = %lf, len3 = %lf\n", len1, len2, len3); */
		/* 	for (i = 0; i < leftcount; ++i)  */
		/* 		show_triangle(parent->tarry, tmp1[i]); */
		/* } */

		/* if (parent->trianglenum > 30) { */
		/* 	printf("parent->trianglenum too large: %d\n", parent->trianglenum); */
		/* } */

		left->tindex = NULL;
		right->tindex = NULL;
		left->trianglenum = 0;
		right->trianglenum = 0;
		free(tmp1);
		free(tmp2);
		
		return 1;
	}
		


	left->tindex = (int *)malloc(sizeof(int) * leftcount);
	if (left->tindex == NULL) {
		left->tindex = NULL;
		right->tindex = NULL;
		leftcount = 0;
		rightcount = 0;

		free(tmp1);
		free(tmp2);

		return 2;
	}

	right->tindex = (int *)malloc(sizeof(int) * rightcount);				
	if (right->tindex == NULL) {
		left->tindex = NULL;
		right->tindex = NULL;
		leftcount = 0;
		rightcount = 0;
		
		free(left->tindex);
		free(tmp1);
		free(tmp2);
		
		return 2;
	}
	
	memcpy(left->tindex, tmp1, sizeof(int) * leftcount);
	memcpy(right->tindex, tmp2, sizeof(int) * rightcount);
	
	left->trianglenum = leftcount;
	right->trianglenum = rightcount;

	free(tmp1);
	free(tmp2);

	return 0;
}

void show_triangle(triangle *t, int index) 
{
	printf("triangle %d:\n", index);
	printf("normal vector: %f, %f, %f\n",
	       t[index].normalvector.x, t[index].normalvector.y, t[index].normalvector.z);
	printf("vertex1: %f, %f, %f\n", t[index].vertex1.x, t[index].vertex1.y, t[index].vertex1.z);
	printf("vertex2: %f, %f, %f\n", t[index].vertex2.x, t[index].vertex2.y, t[index].vertex2.z);
	printf("vertex3: %f, %f, %f\n", t[index].vertex3.x, t[index].vertex3.y, t[index].vertex3.z);
}

void show_volume(const volume *v)
{
	printf("volume:\n");
	printf("xmin = %f, ymin = %f, zmin = %f\n", v->xmin, v->ymin, v->zmin);
	printf("xmax = %f, ymax = %f, zmax = %f\n", v->xmax, v->ymax, v->zmax);
}

//show the volume tree
void show_tree_recur(const volumenode *vnode, int depth)
{
	printf("depth = %d\n", depth);
	
	if (vnode->last == 1)
		printf("triangle num: = %d\n", vnode->trianglenum);

#ifdef SHOWVOLUME	
	show_volume(&vnode->v);
#endif	

	/* if (vnode->child1 != NULL && vnode->child1->trianglenum != 0) { */
	/* 	for (i = 0; i < depth; ++i)  */
	/* 		printf("      "); */

	/* 	printf("%d\n", vnode->child1->trianglenum); */
	/* } */

	/* if (vnode->child2 != NULL && vnode->child2->trianglenum != 0) {  */
	/* 	for (i = 0; i < depth; ++i)  */
	/* 		printf("      "); */

	/* 	printf("%d\n", vnode->child2->trianglenum); */

	
	/* } */
	
	if (vnode->child1 != NULL && vnode->child2 != NULL) {
		show_tree_recur(vnode->child1, depth + 1);
		show_tree_recur(vnode->child2, depth + 1);
	}
}

int collision_detection_recur(const volumenode *left_node, const volumenode *right_node,
							  matrix this_matrix, vector3d *this_vector)
{
	if (left_node == NULL || right_node == NULL) {
		return 0;
	}
	
#ifdef DEBUG
	cdcount++;
#endif
	
	int tmp;
	int i, j;
	int collision_flag = 0;

	volume volume_tmp;
	triangle triangle_tmp;
	
// #ifdef DEBUG
// 	if (left_node->m == NULL || right_node->m == NULL)
// 		printf("error!\n");
// #endif

//	update_volume((double (*)[3])left_node->m, left_node->vector, &left_node->v, &volume_tmp1);
	update_volume(this_matrix, this_vector, &right_node->v, &volume_tmp);
	
//	printf("right_node\n");
//	show_volume(&right_node->v);

	/* printf("left_node\n"); */
	/* show_volume(&volume_tmp2); */

	if (volumecd(&left_node->v, &volume_tmp)) {
		tmp = left_node->last + right_node->last * 2;
		switch(tmp) {
		case 0:
			return (collision_detection_recur(left_node->child1, right_node->child1,
				this_matrix, this_vector) ||
				collision_detection_recur(left_node->child1, right_node->child2,
				this_matrix, this_vector) ||
				collision_detection_recur(left_node->child2, right_node->child1,
				this_matrix, this_vector) ||
				collision_detection_recur(left_node->child2, right_node->child2,
				this_matrix, this_vector));
		case 1:
			return (collision_detection_recur(left_node, right_node->child1,
				this_matrix, this_vector) ||
				collision_detection_recur(left_node, right_node->child2,
				this_matrix, this_vector));
		case 2:
			return (collision_detection_recur(left_node->child1, right_node,
				this_matrix, this_vector) ||
				collision_detection_recur(left_node->child2, right_node,
				this_matrix, this_vector));
		case 3:
#ifdef DEBUG
			if (left_node->trianglenum < 1 || right_node->trianglenum < 1) 
				printf("number error!\n");
#endif		
			for (i = 0; i < left_node->trianglenum; ++i) {
				for (j = 0; j < right_node->trianglenum; ++j) {
					/* triangle_transform((double (*)[3])left_node->m, left_node->vector, */
					/* 		   &left_node->tarry[left_node->tindex[i]], &triangle_tmp1); */					
					
#ifdef DEBUG
					if(left_node->tindex[i] >= left_node->tarraysize) {
						printf("\nindex error\n");
						return 2;
					}
					if(right_node->tindex[j] >= right_node->tarraysize) {
						printf("\nindex error\n");
						return 2;
					}
#endif

					triangle_transform(this_matrix, this_vector,
							   &right_node->tarry[right_node->tindex[j]], &triangle_tmp);
#ifdef DEBUG
					triangle_cd_count++;
#endif
					
					collision_flag = triangle_cd(&(left_node->tarry[left_node->tindex[i]]), &triangle_tmp);					
					if (collision_flag) {
#ifdef DEBUG
						printf("\ntriangle collision!\nindex1 = %d, index2 = %d\n\n",
						       left_node->tindex[i], right_node->tindex[j]);
						show_triangle(left_node->tarry, left_node->tindex[i]);
						show_triangle(right_node->tarry, right_node->tindex[j]);
#endif
						break;
					}
				}
				if (collision_flag) 
					break;
			}

			if (collision_flag) 
				return 1;
			
			return 0;
		default:
#ifdef DEBUG
			printf("vnode->last error!left_node->last = %d, right_node->last = %d\n",
			       left_node->last, right_node->last);
#endif

			return 2;
		}
	}

#ifdef DEBUG
//	printf("novolumecd!\n");
#endif

	return 0;
}

int merge_index_recur(volumenode *vnode, triangle *tarry_new)
{
	vnode->tarraysize = vnode->parent->tarraysize;
	if(vnode->last == 1) {
		int i, j;
		int n = vnode->trianglenum;
		if(n < 1) {
#ifdef DEBUG
			printf("\nerror: trianglenum less than 1\n");
#endif			
			return 1;
		}
		
		int count = 0;
		int *tmp = (int *)malloc(sizeof(int) * n);
		
		for(i = 0; i < n; ++i) {
			for(j = 0; j < count; ++j) {
				if((vnode->tarry[vnode->tindex[i]]).parent == tmp[j]) {
					break;
				}
			}
			if(j == count) {
				tmp[count] = (vnode->tarry[vnode->tindex[i]]).parent;
				count++;
			}
		}
		
		free(vnode->tindex);
		vnode->tindex = (int *)malloc(sizeof(int) * count);
		memcpy(vnode->tindex, tmp, sizeof(int) * count);
		free(tmp);
		vnode->trianglenum = count;

		vnode->tarry = tarry_new;

		return 0;
	}

	vnode->tarry = tarry_new;
	
	if(vnode->child1 == NULL || vnode->child2 == NULL) {
#ifdef DEBUG
		printf("\nerror: no child while last != 1\n");
#endif
		return 1;
	}

	return merge_index_recur(vnode->child1, tarry_new) || merge_index_recur(vnode->child2, tarry_new);	
}

int merge_index(volumenode *vnode, stldata *model)
{
	vnode->tarraysize = model->num;
	
	if(merge_index_recur(vnode, model->ptriangle)) {
#ifdef DEBUG
		printf("\nmerge_index_recur error\n");
#endif
		return 1;
	}
	
	free(model->ptriangle_cutted);
	vnode->tarry = model->ptriangle;
	return 0;
}

volumenode *cd_init(const char *path, const cd_parameter *p)
{
	stldata model;
	volumenode *vnode;
	
	if (loadstl(path, &model) != 0) {
		printf("loadstl %s error!\n", path);
		return NULL;
	}

	if(stl_check(&model)) {
		return NULL;
	}

#ifdef DEBUG	
	printf("model :%s\n", model.modelname);
	printf("triangle number before cut: %d\n", model.num);	
#endif

	if(triangle_cut(&model, p->max_length) == 1) {
		return NULL;
	}

#ifdef DEBUG
	printf("triangle number after cut: %d\n", model.num_cutted);
#endif

	vnode = (volumenode *)malloc(sizeof(volumenode));
	vnode->tarry = model.ptriangle_cutted;
	vnode->tarraysize = model.num_cutted;
	vnode->tindex = (int *)malloc(sizeof(int) * model.num_cutted);

	int i;
	
	for (i = 0; i < model.num_cutted; ++i)
		vnode->tindex[i] = i;
		
	vnode->trianglenum = model.num_cutted;
	build_volume(vnode);
	vnode->parent = vnode;
	
// 	vnode->m = (double *)malloc(sizeof(matrix));
// 	vnode->vector = (vector3d *)malloc(sizeof(vector3d));

	if (build_tree_recur(vnode, p->max_triangle) == -1) 
		printf("build_tree_recur error!\n");
	
#ifdef DEBUG
	printf("\nvolumecount = %d, lastcount = %d\n", volumecount, last_count);
#endif	

	if (merge_index(vnode, &model)) {
		return NULL;
	}
	
#ifdef DEBUG
//	show_tree_recur(vnode, 0);
#endif	
	
	return vnode;
}

int collision_detection1(volumenode *left_node,	volumenode *right_node,
			 matrix r2l_m, vector3d *r2l_v)
{
	matrix this_matrix;
	vector3d this_vector;

	int i, j;
	
	for (i = 0; i < 3; ++i)
		for (j = 0; j < 3; ++j)
				this_matrix[i][j] = r2l_m[i][j];

	if (left_node->trianglenum < right_node->trianglenum) {
		volumenode *swap;
		
		swap = left_node;
		left_node = right_node;
		right_node = swap;
		
		vector3d zero = {0.0, 0.0, 0.0};
		vector3d buf_v;
		
		buf_v.x = -r2l_v->x;
		buf_v.y = -r2l_v->y;
		buf_v.z = -r2l_v->z;
		
		transpose(this_matrix);
		point_transform(this_matrix, &zero, &buf_v, &this_vector);
	} else {
		this_vector.x = r2l_v->x;
		this_vector.y = r2l_v->y;
		this_vector.z = r2l_v->z;
	}

#ifdef DEBUG
		cdcount = 0;
		triangle_cd_count = 0;
#endif
	
	int flag = collision_detection_recur(left_node, right_node, this_matrix, &this_vector);
	
#ifdef DEBUG
	printf("\ncdcount = %d,  triangle_cd_count= %d\n", cdcount, triangle_cd_count);
#endif
	
	return flag;
}

int collision_detection2(volumenode *left_node, matrix m1, vector3d *v1,
			 volumenode *right_node, matrix m2, vector3d *v2)
{
	int i, j;
	matrix tmp_m1;
	matrix tmp_m2;
	vector3d tmp_v1;
	vector3d tmp_v2;

	if (left_node->trianglenum < right_node->trianglenum) {
		volumenode *node_swap;
		vector3d *v_swap;
		
#ifdef DEBUG
		printf("\nmodel swapped\n");
#endif
		node_swap = left_node;
		left_node = right_node;
		right_node = node_swap;

		v_swap = v1;
		v1 = v2;
		v2 = v_swap;

		for (i = 0; i < 3; ++i)
			for (j = 0; j < 3; ++j)
				tmp_m1[i][j] = m2[i][j];

		for (i = 0; i < 3; ++i)
			for (j = 0; j < 3; ++j)
				tmp_m2[i][j] = m1[i][j];
	} else {
		for (i = 0; i < 3; ++i)
			for (j = 0; j < 3; ++j)
				tmp_m1[i][j] = m1[i][j];

		for (i = 0; i < 3; ++i)
			for (j = 0; j < 3; ++j)
				tmp_m2[i][j] = m2[i][j];		
	}

	vector3d buf_v;
//	vector3d buf_v2;
	
	vector3d zero = {0.0, 0.0, 0.0};

	matrix this_matrix;
	vector3d this_vector;

	transpose(tmp_m1);
	matrix_multiply(tmp_m1, tmp_m2, this_matrix);

	tmp_v1.x = -v1->x;
	tmp_v1.y = -v1->y;
	tmp_v1.z = -v1->z;
	tmp_v2.x = v2->x;
	tmp_v2.y = v2->y;
	tmp_v2.z = v2->z;
		
	point_transform(tmp_m1, &zero, &tmp_v1, &buf_v);
	point_transform(tmp_m1, &buf_v, &tmp_v2, &this_vector);

#ifdef DEBUG
	cdcount = 0;
	triangle_cd_count = 0;
#endif

	int flag = collision_detection_recur(left_node, right_node, this_matrix, &this_vector);

#ifdef DEBUG
	printf("\ncdcount = %d,  triangle_cd_count= %d\n", cdcount, triangle_cd_count);
#endif

	return flag;
}

int cd_finish_recur(volumenode *vnode)
{
	if (vnode == NULL)
		return 0;

	if (vnode->last == 1)
		free(vnode->tindex);

	cd_finish_recur(vnode->child1);
	cd_finish_recur(vnode->child2);
	free(vnode);
}

int cd_finish(volumenode *vnode)
{
	if (vnode == NULL)
		return 0;
	free(vnode->tarry);
// 	free(vnode->m);
// 	free(vnode->vector);
	cd_finish_recur(vnode);

	return 0;
}
