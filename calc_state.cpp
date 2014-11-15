#include "state.h"

IKinematicAlg *rob = new KunshanRKA();

int calc_state(state *s, state *pre_s)
{
	input *input = &s->in;
//	printf("seam %d of %d\n", i, n.size());
	solution_array *sa;

	Vector3D _after(0, 0, 1);
	RPY rpy_tmp(0.0, 0.0, 0.0, input->fai2, input->fai1, 0.0);
	TRANS trans_tmp;
	rpy_tmp.RPY2Trans(trans_tmp);
	_after = trans_tmp * _after;

	vector3d before, after;

	before.x = input->n.dx;
	before.y = input->n.dy;
	before.z = input->n.dz;

	after.x = _after.dx;
	after.y = _after.dy;
	after.z = _after.dz;
	
	sa = positioner_inverse(&before, &after, &input->lim[6], &input->lim[7]);
	if(sa != NULL) {
#ifdef DEBUG
		printf("number of solution: %d\n", sa->num);
		for(int i = 0; i < sa->num; ++i) {
			printf("solutions:(%lf, %lf)\n", sa->sol[i].theta1 * 180 / PI, sa->sol[i].theta2 * 180 / PI);
		}
#endif
		
		s->ex_angle.set_angles(sa->sol[0].theta1 * 180 / PI, sa->sol[0].theta2 * 180 / PI, 0.0, 0.0, 0.0, 0.0);
	} else {
#ifdef DEBUG
		printf("error, sa equals NULL\n");
		printf("before = %lf %lf %lf\n", input->n.dx, input->n.dy, input->n.dz);
#endif
		return -1;
	}

	free(sa->sol);
	free(sa);

	Vector3D axis_z;
	axis_z = input->t * input->n;
	TRANS in_part_coord(- input->n.dx, - input->n.dy, - input->n.dz, input->t.dx, input->t.dy, input->t.dz, 
			axis_z.dx, axis_z.dy, axis_z.dz, input->p.dx, input->p.dy, input->p.dz);

	RPY rpy_rotation(0.0, 0.0, 0.0, input->theta, input->pthai, 0.0);
	TRANS rotation;
	rpy_rotation.RPY2Trans(rotation);
	in_part_coord *= rotation;

	s->ex_angle.set_angle(input->x, 3); /* rail */
	
	TRANS part_trans;
	part_trans = Transform::getTransWorldToWorkpiece(s->ex_angle);
			
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

//	delete rob;
	return 0;
}
