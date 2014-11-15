#include <stdio.h>
#include <vector>
using namespace std;

#include "load_seam.h"
#include "positioner.h"
#include "Transform.h"

//#define DEBUG
int load_seam(char *path, vector<Vector3D> &p, vector<Vector3D> &n, vector<Vector3D> &t)
{
	p.clear();
	n.clear();
	t.clear();

	FILE *file;
	if((file = fopen(path, "rb")) == NULL) {
		printf("open file %s error\n", path);
		return -1;
	}
	
	int ret = 0;
	int id = 0;
	
	int seam_num;
	ret = fscanf(file, "%*s%*s%d", &seam_num);
	if (ret != 1) {
		printf("fscanf error 0: ret = %d", ret);
		return -1;
	}
// 	
// 	vector<vector3d> p;
// 	vector<vector3d> n;
// 	vector<vector3d> angle;
	
	for (int j = 0; j < seam_num; j++) {
		int num;
		ret = fscanf(file, "%*s%d%*s%d", &id, &num);
		if (ret != 2) {
			break;
		}

#ifdef DEBUG
		printf("seam %d num %d\n", id, num);
#endif
		for (int i = 0; i < num; i++) {
			Vector3D p_tmp;
			Vector3D t_tmp;
			Vector3D before;
			ret = fscanf(file, "%*s%lf %lf %lf%*s%lf %lf %lf%*s%lf %lf %lf",
				&p_tmp.dx, &p_tmp.dy, &p_tmp.dz, 
				&before.dx, &before.dy, &before.dz,
				&t_tmp.dx, &t_tmp.dy, &t_tmp.dz);
			if (ret != 9) {
				printf("fscanf error 2: ret = %d\n", ret);
				return -1;
			}
			
			p.push_back(p_tmp);
			n.push_back(before);
			t.push_back(t_tmp);
			
#ifdef DEBUG
			printf("P %d: %lf, %lf, %lf N %d: %lf, %lf, %lf T %d: %lf, %lf, %lf\n", 
				i, p_tmp.dx, p_tmp.dy, p_tmp.dz, 
				i, before.dx, before.dy, before.dz,
				i, t_tmp.dx, t_tmp.dy, t_tmp.dz);
#endif
		}
	}
	
// 	vector3d after = {0, 0, 1};
// 	limit lim1 = {-10.0 * DEGREE_TO_RADIAN, 91.0 * DEGREE_TO_RADIAN, 1.0 * DEGREE_TO_RADIAN};
// 	limit lim2 = {-180 * DEGREE_TO_RADIAN, 180.0 * DEGREE_TO_RADIAN, 1.0 * DEGREE_TO_RADIAN};
// 	
// 	for (int i = 0; i < n.size(); i++) {
// 		// 		if (i != 366) {
// 		// 			continue;;
// 		// 		}
// 		printf("seam %d of %d\n", i, n.size());
// 		solution_array *sa;
// 		
// 		sa = positioner_inverse(&n[i], &after, &lim1, &lim2);
// 		if(sa != NULL) {
// 			printf("number of solution: %d\n", sa->num);
// 			for(int i = 0; i < sa->num; ++i) {
// 				printf("solutions:(%lf, %lf)\n", sa->sol[i].theta1 * 180 / PI, sa->sol[i].theta2 * 180 / PI);
// 			}	
// 			
// 			vector3d tmp;
// 			tmp.x = sa->sol[0].theta1 * 180 / PI;
// 			tmp.y = sa->sol[0].theta2 * 180 / PI;
// 			tmp.z = 0.0;
// 			angle.push_back(tmp);
// 		} else {
// 			printf("error, sa equals NULL\n");
// 			printf("i = %d, before = %lf %lf %lf\n", i, n[i].x, n[i].y, n[i].z);
// 			return -1;
// 		}
// 		
// 		free(sa);
// 	}

	return 0;
}
