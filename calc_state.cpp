#include <cmath>
#include <assert.h>
#include "state.h"

IKinematicAlg *rob = new KunshanRKA();

static inline double angle_between(Vector3D& lhs, Vector3D& rhs)
{
	double llen = lhs.get_length();
	double rlen = rhs.get_length();
	double dot = lhs ^ rhs;

	assert(llen > 1e-6 && rlen > 1e-6);
	double tmp = dot / llen / rlen;
	// if(!(tmp <= 1.0 && tmp >= -1.0)) {
	// 	std::cout << "tmp = " << tmp << std::endl;
	// 	std::cout << "llen = " << llen << std::endl;
	// 	std::cout << "rlen = " << rlen << std::endl;
	// 	std::cout << "dot = " << dot << std::endl;
	// }

	assert(tmp <= 1.0 && tmp >= -1.0);
//	return tmp;
	return acos(tmp);
}

int calc_state(state *s, state *pre_s)
{
	input *input = &s->in;
//	printf("seam %d of %d\n", i, n.size());
//	solution_array *sa;

//	Vector3D _after(0, 0, 1);
//	RPY rpy_tm
//	TRANS trans_tmp;
//	rpy_tmp.RPY2Trans(trans_tmp);
//	_after = trans_tmp * _after;

	// vector3d before, after;

	// before.x = input->n.dx;
	// before.y = input->n.dy;
	// before.z = input->n.dz;

	// after.x = _after.dx;
	// after.y = _after.dy;
	// after.z = _after.dz;
	
// 	sa = positioner_inverse(&before, &after, &input->lim[6], &input->lim[7], pre_s->ex_angle);
// 	if(sa != NULL) {
// #ifdef DEBUG
// 		printf("number of solution: %d\n", sa->num);
// 		for(int i = 0; i < sa->num; ++i) {
// 			printf("solutions:(%lf, %lf)\n", sa->sol[i].theta1 * 180 / PI, sa->sol[i].theta2 * 180 / PI);
// 		}
// #endif

// 		if (sa->num > 1) {
// 			std::cout << "sa->num: " << sa->num << std::endl;
// 			for (int i = 0; i < sa->num; i++)
// 				std::cout << "theta1: " << sa->sol[i].theta1 * 180 / PI 
// 					  << "theta2: " << sa->sol[i].theta2 * 180 / PI << std::endl;
// 		}
// 		s->ex_angle.set_angles(sa->sol[0].theta1 * 180 / PI, sa->sol[0].theta2 * 180 / PI, 0.0, 0.0, 0.0, 0.0);
// 	} else {
// #ifdef DEBUG
// 		printf("error, sa equals NULL\n");
// 		printf("before = %lf %lf %lf\n", input->n.dx, input->n.dy, input->n.dz);
// #endif
// 		return -1;
// 	}

// 	free_solution(sa);

	Vector3D axis_z;
	axis_z = input->t * input->n;
	TRANS in_part_coord(- input->n.dx, - input->n.dy, - input->n.dz, 
			    input->t.dx, input->t.dy, input->t.dz, 
			    axis_z.dx, axis_z.dy, axis_z.dz, 
			    input->p.dx, input->p.dy, input->p.dz);

//	 input->pusi = 0.0;
//	 input->theta = 0.0;
//	 input->fi = 0.0;
//	 input->ex1 = 45.0;
//	 input->ex2 = 0.0;
//	 input->ex3 = 0.0;

	RPY rpy_rotation(0.0, 0.0, 0.0, input->fi, input->theta, input->pusi);

	TRANS rotation;
	rpy_rotation.RPY2Trans(rotation);
	in_part_coord *= rotation;

	Vector3D x_gun;
	x_gun.dx = - in_part_coord.rot.mem[0][0];
	x_gun.dy = - in_part_coord.rot.mem[1][0];
	x_gun.dz = - in_part_coord.rot.mem[2][0];

	s->m_cri[3] = angle_between(x_gun, input->n);
//	std::cout << "m_cri[3] = " << s->m_cri[3] << std::endl;

	s->ex_angle.set_angle(input->ex1, 1); /* position 0 */
	s->ex_angle.set_angle(input->ex2, 2); /* position 1 */
	s->ex_angle.set_angle(input->ex3, 3); /* rail 0 */

	TRANS part_trans;
	part_trans = Transform::getTransWorldToWorkpiece(s->ex_angle);

	Vector3D n1, t1;
	Vector3D up(0.0, 0.0, 1.0);
	n1 = part_trans * input->n;
	t1 = part_trans * input->t;
	s->m_cri[4] = angle_between(n1, up);
	s->m_cri[5] = angle_between(t1, up);


	// Vector3D up(0.0, 0.0, 1.0);

	// s->m_cri[4] = cos(angle_between(n1, up));
//	std::cout << "m_cri[4] = " << s->m_cri[4] << std::endl;
			
	TRANS torch_in_world;
	torch_in_world = part_trans * in_part_coord;

					
	TRANS t6_to_torch;
	t6_to_torch = Transform::getTrans6ToTorch();
	t6_to_torch.inverse();
				
	TRANS t6_in_world;
	t6_in_world = torch_in_world * t6_to_torch;
			
	TRANS w_to_base;
	w_to_base = Transform::getTransWorldToBase(s->ex_angle);
	w_to_base.inverse();
			
	TRANS t6_in_robot;
	t6_in_robot = w_to_base * t6_in_world;
	
	if (rob->InverseRobot(s->angle, pre_s->angle, t6_in_robot)) {
#ifdef DEBUG
		printf("error inverserobot\n");
#endif
//		delete rob;
		return -1;
	}
	// to_continuous(s->angle, pre_s->angle);
	// to_continuous(s->ex_angle, pre_s->ex_angle);
//	delete rob;
	return 0;
}
