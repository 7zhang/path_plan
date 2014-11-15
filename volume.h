#ifndef _VOLUME_H_
#define _VOLUME_H_

#include "geom.h"

typedef struct _volume {
	double xmin, xmax, ymin, ymax, zmin, zmax;
}volume;

typedef struct _volumenode {
//const element during collision detection
	triangle *tarry;
	int tarraysize;
	int *tindex;
	int trianglenum;
	int depth;
	volume v;
	int last;

	struct _volumenode *parent;
	struct _volumenode *child1, *child2;
	/* struct _volumenode *left, *right; */

	double *m;
	vector3d *vector;

/* //mutable element during collision detection */
/* 	int cdflag; */
/* 	struct _volumenode *cdvolumelist; */
/* 	int volumenum; */
}volumenode;

typedef struct _parameter
{
	int max_triangle;
	int max_length;
}parameter;

// cd initialization
// extern "C" __declspec(dllexport) volumenode *cd_init(char *path, parameter *p);
// 
// /* collision detection 1*/
// /* use one transformation matrix to transform righ_node into left_node's coordinate */
// /* not tested, bug may exist */
// extern "C" __declspec(dllexport) int collision_detection1(volumenode *left_node,	volumenode *right_node,
// 			 matrix r2l_m, vector3d *r2l_v);
// 
// /* collision detection 2*/
// /* use two transformation matrix related to the same world coordinate */
// extern "C" __declspec(dllexport) int collision_detection2(volumenode *left_node, matrix m1, vector3d *v1,
// 			 volumenode *right_node, matrix m2, vector3d *v2);
// 
// finish cd, release memory
// extern "C" __declspec(dllexport) int cd_finish(volumenode *vnode);
// 
// show triangle data
// extern "C" __declspec(dllexport) void show_triangle(triangle *t, int index);
// 
// show the volume tree
// extern "C" __declspec(dllexport) void show_tree_recur(const volumenode *vnode, int depth);

#endif /* _VOLUME_H_ */
