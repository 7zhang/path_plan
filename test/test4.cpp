#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include "volume.h"
#include "load_seam.h"
#include "Transform.h"
#include "KunShanJacobi.h"
#include "state.h"
#include "calc_criteria.h"
#include "thread.h"

// 
// #ifdef DEBUG
// int volumecount = 0;
// int cdcount = 0;
// int last_count = 0;
// int triangle_cd_count = 0;
// #endif

typedef volumenode *(*cdinit)(char *path, parameter *p);
typedef int (*cd1)(volumenode *left_node,	volumenode *right_node,
			 matrix r2l_m, vector3d *r2l_v);
typedef int (*cd2)(volumenode *left_node, matrix m1, vector3d *v1,
			 volumenode *right_node, matrix m2, vector3d *v2);
typedef int (*cdfinish)(volumenode *vnode);

int program_jpos(vector<JAngle> &angle, vector<JAngle> &ex_angle, char *path);
int program_cpos(vector<RPY> &rpy, vector<JAngle> &ex_angle, char *path);
void to_continuous(JAngle &angle, JAngle &last);

#define DIMENSION1 5
#define DIMENSION2 36
#define DIMENSION3 200

int main(int argc, char *argv[])
{
// 	HINSTANCE hinstance = LoadLibrary("cd.dll");
// 
// 	if (NULL == hinstance)
// 	{
// 		printf("error\n");
// 		exit(1);
// 	}
// 	cdinit cd_init = (cdinit)GetProcAddress(hinstance, "cd_init");
// 	cd2 collision_detection2 = (cd2)GetProcAddress(hinstance, "collision_detection2");
// 	cdfinish cd_finish = (cdfinish)GetProcAddress(hinstance, "cd_finish");
// 	volumenode *left_node, *right_node;
// 
// 	parameter p;
// 	p.max_length = atoi(argv[9]);
// 	p.max_triangle = atoi(argv[10]);
// 
// 	left_node = cd_init(argv[1], &p);
// 	if(left_node == NULL) {
// 		printf("cd_init error\n");
// 		exit(1);
// 	}
// 
// 	right_node = cd_init(argv[2], &p);
// 	if(right_node == NULL) {
// 		printf("cd_init error\n");
// 		exit(1);
// 	}
// 
// 	clock_t start, end;
// 	start = clock();
// 	
// 	JAngle robotangle(atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]),
// 		  atof(argv[7]), atof(argv[8]));
// 	
// 	/* JAngle robotangle(0.0, -23.00, 52.500, 16.00, 19.00, 0.00); */
// 	JAngle exangle(0.00, 0.00, 0.00, 0.00, 0.00, 0.00);
// 	
// 
// 	TRANS j0_trans = Transform::getTransWorldToBase(exangle);
// 	TRANS j1_trans, j2_trans, j3_trans, j4_trans, j5_trans, j6_trans;
// 	TRANS part_trans, newgun_trans;
// 
// 	Transform::getTransBaseToJoints(robotangle, j1_trans, j2_trans, j3_trans, j4_trans, j5_trans, j6_trans);
// 	newgun_trans = Transform::getTrans6ToGun();
// 	
// 	j1_trans = j0_trans * j1_trans;
// 	j2_trans = j0_trans * j2_trans;
// 	j3_trans = j0_trans * j3_trans;
// 	j4_trans = j0_trans * j4_trans;
// 	j5_trans = j0_trans * j5_trans;
// 	j6_trans = j0_trans * j6_trans;
// 	newgun_trans = j6_trans * newgun_trans;
// 	
// 	part_trans = Transform::getTransWorldToWorkpiece(exangle);
// 	
// 	vector3d tmp1, tmp2;
// 	tmp1.x = newgun_trans.pos.dx;
// 	tmp1.y = newgun_trans.pos.dy;
// 	tmp1.z = newgun_trans.pos.dz;
// 	
// 	tmp2.x = part_trans.pos.dx;
// 	tmp2.y = part_trans.pos.dy;
// 	tmp2.z = part_trans.pos.dz;
// 
// 	int i;
// 
// 	for (i = 0; i < 1; ++i) {
// 		if (collision_detection2(right_node, part_trans.rot.mem, &tmp2, left_node, newgun_trans.rot.mem, &tmp1)) {
// 			printf("\ncollision detected!\n\n");
// 		} else {
// 			printf("\nno collision!\n\n");
// 		}
// 		/* collision_detection2(left_top_node, right_top_node); */
// 	}
// 	
// 	end = clock();
// 	
// 	printf("run %d times, time elapsed %f\n", i, (double)(end - start) / CLOCKS_PER_SEC);
// 
// #ifdef DEBUG
// 	printf("cdcount = %d\n", cdcount);
// 
// 	printf("triangle_cd_count = %d\n", triangle_cd_count);
// #endif
// 
// #ifdef SHOW
// 	show_tree_recur(left_node, 0);
// 	printf("rightnode\n");
// 	show_tree_recur(right_node, 0);
// #endif
// 	
// 	cd_finish(left_node);
// 	cd_finish(right_node);

// 	vector<vector3d> normal, tangent, point, positioner_angle;
// 	if (load_seam("test1.pos", point, normal, tangent, positioner_angle)) {
// 		printf("load_seam error\n");
// 		return -1;
// 	}
// 
// 	JAngle angle;
// 	JAngle last(0.0, -90.00, 90.00, 0.0, 0.0, 0.0);
// 	vector<JAngle> v_angle, v_ex_angle;
// 	vector<RPY> v_rpy;
// 
// 	TRANS part_trans;
// 
// 	int error_count = 0;
// 
// 	double *result = (double *)malloc(sizeof(double) * DIMENSION1 * DIMENSION2 * DIMENSION3);
// 	for (int count = 0; count < DIMENSION1/*positioner_angle.size()*/; count++) {
// 		count = count * 10;
// 		vector3d tmp = positioner_angle[count];
// 		vector3d p_weld = point[count];
// 		vector3d n_weld = normal[count];
// 		vector3d t_weld = tangent[count];
// 		count = count / 10;
// 		vector3d axis_z;
// 		vectorcross(&t_weld, &n_weld, &axis_z);
// 		TRANS in_part_coord(- n_weld.x, - n_weld.y, - n_weld.z, t_weld.x, t_weld.y, t_weld.z, 
// 			axis_z.x, axis_z.y, axis_z.z, p_weld.x, p_weld.y, p_weld.z);
// 
// 		double max = 0;
// 		for (int j = 0; j < DIMENSION2; j++) {
// 			RPY rpy_rotation(j - DIMENSION2 / 2, 0.0, 0.0, 0.0, 0.0, 0.0);
// 			TRANS rotation;
// 			rpy_rotation.RPY2Trans(rotation);
// 			in_part_coord *= rotation;
// 			
// 			RPY rpy_tmp;
// 			in_part_coord.Trans2RPY(rpy_tmp);
// 			v_rpy.push_back(rpy_tmp);
// 			JAngle rob_angle, ex_angle;
// 			for (int k = 0; k < DIMENSION3; k++) {
// 				ex_angle.set_angles(tmp.x, tmp.y, (k - DIMENSION3 / 2) * (2000 / DIMENSION3), 0.0, 0.0, 0.0);
// 				
// 				part_trans = Transform::getTransWorldToWorkpiece(ex_angle);
// 				
// 				TRANS torch_in_world;
// 				torch_in_world = part_trans * in_part_coord;
// 				
// 				TRANS t6_to_torch;
// 				t6_to_torch = Transform::getTrans6ToTorch();
// 				t6_to_torch.inverse();
// 				
// 				TRANS t6_in_world;
// 				t6_in_world = torch_in_world * t6_to_torch;
// 				
// 				TRANS w_to_base;
// 				w_to_base = Transform::getTransWorldToBase(ex_angle);
// 				w_to_base.inverse();
// 				
// 				TRANS t6_in_robot;
// 				t6_in_robot = w_to_base * t6_in_world;
// 				
// 				IKinematicAlg *rob = new KunshanRKA();
// 				// 	JAngle pos_angle(0.0, -90.0, 90.0, 0, 0, 0);
// 				// 	TRANS pos_t;
// 				// 	rob->PositiveRobot(pos_angle, pos_t);
// 				// 	RPY pos_rpy = pos_t.GetRPY();
// 				// 	RPY rpy(912.02999877616321, 0.0, 1343.41, 180.0, -90.0, 0.0);
// 				// 	TRANS t;
// 				// 	rpy.RPY2Trans(t);
// 				
// 				double value;
// 				if (rob->InverseRobot(angle, last, t6_in_robot)) {
// 					printf("inverserobot error count = %d / %d, j = %d / %d, k = %d / %d\n", count, DIMENSION1, j, DIMENSION2, k, DIMENSION3);
// 					error_count++;
// 					value = 0;
// 					//			return -1;
// 				} else {
// 					to_continuous(angle, last);
// 					
// 					double j[6][6];
// 					
// 					jacobi(j, angle.get_angle(1), angle.get_angle(2), angle.get_angle(3), 
// 						angle.get_angle(4), angle.get_angle(5), angle.get_angle(6));
// 					
// 					value = determinant(j);
// 					
// 					last = angle;
// 				}
// 
// 				result[count * DIMENSION2 * DIMENSION3 + j * DIMENSION3 + k] = value;
// 
// 				double fvalue = fabs(value);
// 				if (fvalue > max) {
// 					max = fvalue;
// 				}
// 			}
// 		}
// 
// 		
// // 		printf("angle: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", angle.get_angle(1), angle.get_angle(2), 
// // 			angle.get_angle(3), angle.get_angle(4), angle.get_angle(5), angle.get_angle(6), ex_angle.get_angle(1),
// // 			ex_angle.get_angle(2), ex_angle.get_angle(3));
// // 				
// // 		v_angle.push_back(angle);
// // 		v_ex_angle.push_back(ex_angle);
// // 
// // 		last = angle;
// 	}
// // 
// // 	program_jpos(v_angle, v_ex_angle, "./program_jpos.glp");
// // 	program_cpos(v_rpy, v_ex_angle, "./program_cpos.glp");
// 
// 	FILE *file;
// 	if((file = fopen("F://jacobi_data.txt", "wb")) == NULL) {
// 		printf("open file %s error\n", "F://jacobi_data.txt");
// 		return -1;
// 	}
// 
// 	for (int ii = 0; ii < DIMENSION1; ii++) {
// 		for (int j = 0; j < DIMENSION2; j++) {
// 			for (int k = 0; k < DIMENSION3; k++) {
// 				fprintf(file, "%lf\n", result[ii * DIMENSION2 * DIMENSION3 + j * DIMENSION3 + k]);
// 			}
// 		}
// 	}
// 
// 	fclose(file);
// 	printf("error_count = %d\n", error_count);

	vector<Vector3D> normal, tangent, point;
	if (load_seam("test1.pos", point, normal, tangent)) {
		printf("load_seam error\n");
		return -1;
	}

	state s;
	state pre_s;
	limit lim1 = {-10.0 * DEGREE_TO_RADIAN, 91.0 * DEGREE_TO_RADIAN, 1.0 * DEGREE_TO_RADIAN};
	limit lim2 = {-180 * DEGREE_TO_RADIAN, 180.0 * DEGREE_TO_RADIAN, 1.0 * DEGREE_TO_RADIAN};
	s.in.lim[6].max = 91.0 * DEGREE_TO_RADIAN;
	s.in.lim[6].min = -10.0 * DEGREE_TO_RADIAN;
	s.in.lim[6].step = 1.0 * DEGREE_TO_RADIAN;

	s.in.lim[7].max = 180.0 * DEGREE_TO_RADIAN;
	s.in.lim[7].min = -180 * DEGREE_TO_RADIAN;
	s.in.lim[7].step = 1.0 * DEGREE_TO_RADIAN;

	int error_count = 0;
// 	int index;
// 	for (index = 0; index < 5; index++)
// 		s.in.var[index] = 0.0;

	s.in.x = 0.0;
	s.in.theta = 0;
	s.in.pthai = 0;
	s.in.fai1 = 0.0;
	s.in.fai2 = 20;

	vector<JAngle> angle, ex_angle;

	pre_s.angle.set_angles(0.0, -90.00, 90.00, 0.0, 0.0, 0.0);
	pre_s.ex_angle.set_angles(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	clock_t start, end;
	criteria c;
	start = clock();

// 	for (int j = 0; j < 100000; j++) {
// 		s.in.n = normal[0];
// 		s.in.t = tangent[0];
// 		s.in.p = point[0];
// 
// 		calc_state(&inp, &s, &pre_s);
// 		calc_criteria(&inp, &s, &c);
// 	}

	s.in.n = normal[0];
	s.in.t = tangent[0];
	s.in.p = point[0];

	crew_t *crew = create_crew(24, 100, calc_state, &pre_s);
	int state_count = 0;
	int pcount = 0;
	state *states;
	
	for (int x = -1000; x < 1000; x = x + 20) {
		printf("x = %d / %d\n", (1000 + x) / 20, 2000 / 20);
		printf("pcount = %d\n", pcount);
		for (int theta = -180; theta < 180; theta = theta + 5) {
			log("theta = %d / %d\n", theta, 360 / 5);
			for (int pthai = -20; pthai < 20; pthai = pthai + 1) {
				log("pthai = %d / %d\n", pthai, 40 / 1);
				for (int fai1 = -45; fai1 < 45; fai1 = fai1 + 2) {
					log("fai1 = %d / %d\n", fai1, 90 / 2);
					for (int fai2 = -45; fai2 < 45; fai2 = fai2 + 2) {
						log("fai2 = %d / %d\n", fai2, 90 / 2);
						if (state_count == 0) {
							states = (state *)malloc(sizeof(state) * WORK_SIZE);
							if (states == NULL) {
								printf("malloc error!\n");
								return -1;
							}
						}
						
						s.in.x = x;
						s.in.theta = theta;
						s.in.pthai = pthai;
						s.in.fai1 = fai1;
						s.in.fai2 = fai2;

						memcpy(states + state_count, &s, sizeof(state));
						state_count++;
						pcount++;
						
						if (state_count == WORK_SIZE) {
							state_count = 0;
							insert_work(crew, WORK_SIZE, states, calc_criteria);
						}
						// if (calc_state(&s, &pre_s)) {
						// 	continue;
						// }
						// calc_criteria(&s, &c);

						// if (fabs(c.j) > max_s.c.j) {
						// 	max_s.in.x = s.in.x;
						// 	max_s.in.theta = s.in.theta;
						// 	max_s.in.pthai = s.in.pthai;
						// 	max_s.in.fai1 = s.in.fai1;
						// 	max_s.in.fai2 = s.in.fai2;
						// 	max_s.c.j = fabs(c.j);
						// 	max_s.angle = s.angle;
						// 	max_s.ex_angle = s.ex_angle;
						// }
					}
				}
			}
		}
	}

	state *best = done(crew);
	if (best == NULL) {
		printf("error! done return NULL\n");
		return -1;
	}

	end = clock();
	printf("run %d times, time elapsed %f\n", pcount, (double)(end - start) / CLOCKS_PER_SEC);

	printf("max = %lf\nmax_angle = %lf, %lf, %lf, %lf, %lf, %lf\nmax_ex_angle = %lf, %lf, %lf\n",
	       best->c.j, best->angle.get_angle(1), best->angle.get_angle(2), best->angle.get_angle(3), 
	       best->angle.get_angle(4), best->angle.get_angle(5), best->angle.get_angle(6),
	       best->ex_angle.get_angle(1), best->ex_angle.get_angle(2), best->ex_angle.get_angle(3));

	vector<input> best_in;
	vector<JAngle> best_angle;
	vector<JAngle> best_ex_angle;

	best_in.push_back(best->in);
	best_angle.push_back(best->angle);
	best_ex_angle.push_back(best->ex_angle);

	start = clock();
	input pre_inp;

	FILE *file;
	if((file = fopen("./output.log", "wb")) == NULL) {
		printf("open file %s error\n", "./output.log");
		return -1;
	}

	// best->in.x = 0;
	// best->in.theta = 0;
	// best->in.pthai = 0;
	// best->in.fai1 = 0;
	// best->in.fai2 = 0;
	// best->angle = pre_s.angle;
	// best->ex_angle = pre_s.ex_angle;

	for (int count = 0; count < normal.size(); count++) {
		fprintf(file, "count = %d\n", count);
		printf("count = %d / %d\n", count, normal.size());
		s.in.n = normal[count];
		s.in.t = tangent[count];
		s.in.p = point[count];

		pre_s.in.x = best->in.x;
		pre_s.in.theta = best->in.theta;
		pre_s.in.pthai = best->in.pthai;
		pre_s.in.fai1 = best->in.fai1;
		pre_s.in.fai2 = best->in.fai2;
		pre_s.angle = best->angle;
		pre_s.ex_angle = best->ex_angle;

		free(best);

		reset_crew(crew, &pre_s);
		
		state_count = 0;
		pcount = 0;

		int x, theta, pthai, fai1, fai2;
		for (x = - 20; x < 20; x++) {
			printf("x = %d / %d\n", x, 40 / 1);
			for (theta = -15; theta < 15; theta++) {
				log("theta = %d / %d\n", theta, (int)(6 / 0.2));
				for (pthai = -15; pthai < 15; pthai++) {
					log("pthai = %d / %d\n", pthai, (int)(6 / 0.2));
					for (fai1 = -15; fai1 < 15; fai1++) {
						log("fai1 = %d / %d\n", fai1, (int)(6 / 0.2));
						for (fai2 = -15; fai2 < 15; fai2++) {
							log("fai2 = %d / %d\n", fai2, (int)(6 / 0.2));
							if (state_count == 0) {
								states = (state *)malloc
									(sizeof(state) * WORK_SIZE);
								if (states == NULL) {
									printf("malloc error!\n");
									return -1;
								}
							}
						
							s.in.x = pre_s.in.x + x;
							s.in.theta = pre_s.in.theta + theta * 0.2;
							s.in.pthai = pre_s.in.pthai + pthai * 0.2;
							s.in.fai1 = pre_s.in.fai1 + fai1 * 0.2;
							s.in.fai2 = pre_s.in.fai2 + fai2 * 0.2;

							memcpy(states + state_count, &s, sizeof(state));
							state_count++;
							pcount++;
						
							if (state_count == WORK_SIZE) {
								insert_work(crew, state_count, states, calc_criteria);
								state_count = 0;
								log("insert work\n");
							}
						}
					}
				}
			}
		}

		if (state_count > 0) {
			insert_work(crew, state_count, states, calc_criteria);
			log("insert work\n");
		}
		
		printf("state_count = %d, pcount = %d\n", state_count, pcount);

		best = done(crew);
		if (best == NULL) {
			printf("error! done return NULL\n");
			continue;
		}
		
		best_in.push_back(best->in);
		fprintf(file, "max_inp = %lf %lf %lf %lf %lf\n",
			best->in.x, best->in.theta, best->in.pthai,
			best->in.fai1, best->in.fai2);
		printf("max_inp = %lf %lf %lf %lf %lf\nmax_value = %lf\n",
			best->in.x, best->in.theta, best->in.pthai,
		       best->in.fai1, best->in.fai2, best->c.j);
		printf("max_angle = %lf, %lf, %lf, %lf, %lf, %lf\nmax_ex_angle = %lf, %lf, %lf\n",
		       best->angle.get_angle(1), best->angle.get_angle(2), best->angle.get_angle(3), 
		       best->angle.get_angle(4), best->angle.get_angle(5), best->angle.get_angle(6),
		       best->ex_angle.get_angle(1), best->ex_angle.get_angle(2),
		       best->ex_angle.get_angle(3));

		best_angle.push_back(best->angle);
		best_ex_angle.push_back(best->ex_angle);
	}

// 	for (int i = 0; i < normal.size(); i++) {
// 		s.in.n = normal[i];
// 		s.in.t = tangent[i];
// 		s.in.p = point[i];
// 		
// 		printf("i = %d / %d\n", i, normal.size());
// 		if (calc_state(&inp, &s, &pre_s)) {
// 			printf("calc_state error\n");
// 			error_count++;
// 		} else {
// 			to_continuous(s.angle, pre_s.angle);
// 			angle.push_back(s.angle);
// 			ex_angle.push_back(s.ex_angle);
// 
// 			pre_s.angle = s.angle;
// 			pre_s.ex_angle = s.ex_angle;
// 		}
// 	}
// 
// 	printf("error_count = %d\n", error_count);
// 
 	program_jpos(best_angle, best_ex_angle, "./program.glp");

	end = clock();

	printf("run %d times, time elapsed %f\n", normal.size() * 40 * 30 * 30 * 30 * 30, (double)(end - start) / CLOCKS_PER_SEC);
	
	return 0;
}

int program_jpos(vector<JAngle> &angle, vector<JAngle> &ex_angle, char *path)
{
	FILE *file;
	if((file = fopen(path, "wb")) == NULL) {
		printf("open file %s error\n", path);
		return -1;
	}
	
	fwrite("//DATASEG\n", 1, 10, file);
	for (int i = 0; i < angle.size(); i++) {
		fprintf(file, "JPOS: loc%d=(%f,%f,%f,%f,%f,%f,%f,%f,%f)\n",
			i + 1, angle[i].get_angle(1), angle[i].get_angle(2), angle[i].get_angle(3),
			angle[i].get_angle(4), angle[i].get_angle(5), angle[i].get_angle(6),
			ex_angle[i].get_angle(1), ex_angle[i].get_angle(2), ex_angle[i].get_angle(3));
	}
	
	fwrite("//PROGRAMSEG\n", 1, 13, file);
	
	for (int j = 0; j < angle.size(); j++) {
		fprintf(file, "MOVJ (loc%d,Vel=5.000,Acc=100.000,Jerk=100.000)\n", j + 1);
	}
	
	return 0;
}

int program_cpos(vector<RPY> &rpy, vector<JAngle> &ex_angle, char *path)
{
	FILE *file;
	if((file = fopen(path, "wb")) == NULL) {
		printf("open file %s error\n", path);
		return -1;
	}
	
	fwrite("//DATASEG\n", 1, 10, file);
	for (int i = 0; i < rpy.size(); i++) {
		fprintf(file, "CPOS: loc%d=(%f,%f,%f,%f,%f,%f,%f,%f,%f)\n",
			i + 1, rpy[i].pos.dx, rpy[i].pos.dy, rpy[i].pos.dz,
			rpy[i].orient.dx, rpy[i].orient.dy, rpy[i].orient.dz,			
			ex_angle[i].get_angle(1), ex_angle[i].get_angle(2), ex_angle[i].get_angle(3));
	}
	
	fwrite("//PROGRAMSEG\n", 1, 13, file);
	
	for (int j = 0; j < rpy.size(); j++) {
		fprintf(file, "MOVL (loc%d,Vel=5.000,Acc=100.000,Jerk=100.000)\n", j + 1);
	}
	
	return 0;
}

void to_continuous(JAngle &angle, JAngle &last)
{
	if (angle.get_angle(1) - last.get_angle(1) > 90) {
		angle.set_angle(angle.get_angle(1) - 2 * 180, 1);
	} else if (angle.get_angle(1) - last.get_angle(1) < -90) {
		angle.set_angle(angle.get_angle(1) + 2 * 180, 1);
	}

	if (angle.get_angle(2) - last.get_angle(2) > 90) {
		angle.set_angle(angle.get_angle(2) - 2 * 180, 2);
	} else if (angle.get_angle(2) - last.get_angle(2) < -90) {
		angle.set_angle(angle.get_angle(2) + 2 * 180, 2);
	}

	if (angle.get_angle(3) - last.get_angle(3) > 90) {
		angle.set_angle(angle.get_angle(3) - 2 * 180, 3);
	} else if (angle.get_angle(3) - last.get_angle(3) < -90) {
		angle.set_angle(angle.get_angle(3) + 2 * 180, 3);
	}

	if (angle.get_angle(4) - last.get_angle(4) > 90) {
		angle.set_angle(angle.get_angle(4) - 2 * 180, 4);
	} else if (angle.get_angle(4) - last.get_angle(4) < -90) {
		angle.set_angle(angle.get_angle(4) + 2 * 180, 4);
	}

	if (angle.get_angle(5) - last.get_angle(5) > 90) {
		angle.set_angle(angle.get_angle(5) - 2 * 180, 5);
	} else if (angle.get_angle(5) - last.get_angle(5) < -90) {
		angle.set_angle(angle.get_angle(5) + 2 * 180, 5);
	}

	if (angle.get_angle(6) - last.get_angle(6) > 90) {
		angle.set_angle(angle.get_angle(6) - 2 * 180, 6);
	} else if (angle.get_angle(6) - last.get_angle(6) < -90) {
		angle.set_angle(angle.get_angle(6) + 2 * 180, 6);
	}
}

