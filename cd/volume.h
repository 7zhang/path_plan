#ifndef _VOLUME_H_
#define _VOLUME_H_

#include "cd.h"
	
//cd initialization
extern "C" __declspec(dllexport) volumenode *cd_init(char *path, cd_parameter *p);

/* collision detection 1*/
/* use one transformation matrix to transform righ_node into left_node's coordinate */
/* not tested, bug may exist */
extern "C" __declspec(dllexport) int collision_detection1(volumenode *left_node,	volumenode *right_node,
			 matrix r2l_m, vector3d *r2l_v);

/* collision detection 2*/
/* use two transformation matrix related to the same world coordinate */
extern "C" __declspec(dllexport) int collision_detection2(volumenode *left_node, matrix m1, vector3d *v1,
			 volumenode *right_node, matrix m2, vector3d *v2);

//finish cd, release memory
extern "C" __declspec(dllexport) int cd_finish(volumenode *vnode);

#endif /* _VOLUME_H_ */
