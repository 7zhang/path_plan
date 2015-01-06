#ifndef _CD_H_
#define _CD_H_

typedef struct _vector3d
{
	double x;
	double y;
	double z;
}vector3d;

typedef double matrix[3][3];

typedef struct _triangle
{
	vector3d normalvector;
	vector3d vertex1;
	vector3d vertex2;
	vector3d vertex3;

	int parent;
/*	unsigned short attr;*/
}triangle;

typedef struct _stldata
{
	char modelname[80];
	int num;
	triangle *ptriangle;
	int num_cutted;
	triangle *ptriangle_cutted;
}stldata;

typedef struct _volume {
	double xmin, xmax, ymin,
		ymax, zmin, zmax;
}volume;

typedef struct _volumenode {
	triangle *tarry;
	int tarraysize;
	int *tindex;
	int trianglenum;
//	int depth;
	volume v;
	int last;

	struct _volumenode *parent;
	struct _volumenode *child1, *child2;
// 
// 	double *m;
// 	vector3d *vector;

}volumenode;

typedef struct _cd_parameter
{
	int max_triangle;
	int max_length;
}cd_parameter;

extern "C" {
//cd initialization
__attribute__ ((visibility("default"))) volumenode *cd_init(char *path, cd_parameter *p);

/* collision detection 1*/
/* use one transformation matrix to transform righ_node into left_node's coordinate */
/* not tested, bug may exist */
__attribute__ ((visibility("default"))) int collision_detection1(volumenode *left_node,	volumenode *right_node,
						 matrix r2l_m, vector3d *r2l_v);

/* collision detection 2*/
/* use two transformation matrix related to the same world coordinate */
__attribute__ ((visibility("default"))) int collision_detection2(volumenode *left_node, matrix m1, vector3d *v1,
						 volumenode *right_node, matrix m2, vector3d *v2);

//finish cd, release memory
__attribute__ ((visibility("default"))) int cd_finish(volumenode *vnode);
}
#endif /* _CD_H_ */
