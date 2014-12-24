#include "KunShan_robot.h"
#include "state.h"
#include "Transform.h"
#include "calc_criteria.h"
#include "teach_point.h"

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
	double pi = boost::math::constants::pi<double>();
	assert(tmp <= 1.0 && tmp >= -1.0);
//	return tmp;
	return acos(tmp) / pi * 180.0;
}

/*
 * nine axes
 * six auxiliary variable: rpy and jacobi determinant
 * 
 *
 *
 */

void kunshan_robot::init()
{
	m_axis_nr = 9;
	m_axes.resize(9);
	m_axes[0] = axis(-150.0, 180.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[1] = axis(-125.0, 30.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[2] = axis(-120.0, 150.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[3] = axis(-180.0, 180.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[4] = axis(-120.0, 120.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[5] = axis(-180.0, 180.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[6] = axis(0.0, 90.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[7] = axis(-185.0, 185.0, 50.0, 10.0, 10, 0, 0.1);
	m_axes[8] = axis(-1700.0, 1600.0, 50.0, 10.0, 10, 0, 1.0);

	m_auxiliary_variable_nr = 6;
	m_auxiliary_variable.resize(6);
	m_auxiliary_variable[0] = axis(-30.0, 0.0, 3.0, 3.0, 10, 0, 1.0); 
	m_auxiliary_variable[1] = axis(-15.0, 15.0, 3.0, 3.0, 10, 0, 1.0); 
	m_auxiliary_variable[2] = axis(-180.0, 180.0, 3.0, 3.0, 10, 0, 1.0); 
	m_auxiliary_variable[3] = axis(0.0, 1.0, 3.0, 3.0, 10, 3, 1.0);
	m_auxiliary_variable[4] = axis(0.0, 15.0, 3.0, 3.0, 10, 0, 3.0);
	m_auxiliary_variable[5] = axis(75.0, 105.0, 3.0, 3.0, 10, 0, 3.0);

	m_map.push_back(6);
	m_map.push_back(7);
	m_map.push_back(8);
	m_map.push_back(9);
	m_map.push_back(10);
	m_map.push_back(11);

	std::vector<double> mu_axis(9);
	std::vector<double> sigma_axis(9);
	std::vector<double> offset_axis(9, 0);
	std::vector<double> weight_axis(9, 1.0);
	
	for (int i = 0; i < m_axes.size(); i++) {
		mu_axis[i] = m_axes[i].mid();
		sigma_axis[i] = m_axes[i].length() / 6.0;
	}

	teach_point t1(mu_axis, sigma_axis, offset_axis, weight_axis, mu_axis, sigma_axis, offset_axis, weight_axis);
	m_teach_points.push_back(t1);
	m_weight.push_back(1.0);
}

double kunshan_robot::operator() (de::DVectorPtr args) {
	for (int i = 0; i < (*args).size(); i++) {
		set_vars(i, (*args)[i]);
	}
	// state s;
	// state pre_s;

	JAngle pre_angle = JAngle(m_axes[0].last(), m_axes[1].last(), m_axes[2].last(),
			     m_axes[3].last(), m_axes[4].last(), m_axes[5].last());
//	JAngle pre_ex_angle = JAngle(m_axes[6].last(), m_axes[7].last(), m_axes[8].last(),
//				0.0, 0.0, 0.0);
	// m_s.in.ex1 = (*args)[0];
	// m_s.in.ex2 = (*args)[1];
	// m_s.in.ex3 = (*args)[2];
	// m_s.in.pusi = (*args)[3];
	// m_s.in.theta = (*args)[4];
	// m_s.in.fi = (*args)[5];

	TRANS gun_in_seam(0.0, 0.0, -1.0, 
			  1.0, 0.0, 0.0, 
			  0.0, -1.0, 0.0,
			  0.0, 0.0, 0.0);
	TRANS rotateY(cos((*args)[3]), 0.0, -sin((*args)[3]),
		      0.0, 1.0, 0.0,
		      sin((*args)[3]), 0.0, cos((*args)[3]),
		      0.0, 0.0, 0.0);
	TRANS rotateX(1.0, 0.0, 0.0,
		      0.0, cos((*args)[4]), sin((*args)[4]),
		      0.0, -sin((*args)[4]), cos((*args)[4]),
		      0.0, 0.0, 0.0);

	TRANS rotateZ(cos((*args)[5]), sin((*args)[5]), 0.0,
		      -sin((*args)[5]), cos((*args)[5]), 0.0,
		      0.0, 0.0, 1.0,
		      0.0, 0.0, 0.0);
	gun_in_seam = rotateX * rotateY * gun_in_seam * rotateX;

//	print_trans("gun_in_seam", gun_in_seam);
	
	Vector3D axis_y = m_n * m_t;
	TRANS seam_in_part(m_t.dx, m_t.dy, m_t.dz,
			   axis_y.dx, axis_y.dy, axis_y.dz,
			   m_n.dx, m_n.dy, m_n.dz,
			   m_p.dx, m_p.dy, m_p.dz);
	TRANS gun_in_part = seam_in_part * gun_in_seam;

//	print_trans("gun_in_part", gun_in_part);

	JAngle ex_angle((*args)[0], (*args)[1], (*args)[2], 0.0, 0.0, 0.0);
	TRANS part_trans;
	part_trans = Transform::getTransWorldToWorkpiece(ex_angle);

	Vector3D n1, t1;
	Vector3D up(0.0, 0.0, 1.0);
	n1 = part_trans * m_n;
	t1 = part_trans * m_t;
	m_auxiliary_variable_values[4] = angle_between(n1, up);
	m_auxiliary_variable_values[5] = angle_between(t1, up);
				
	TRANS torch_in_world;
	torch_in_world = part_trans * gun_in_part;

//	print_trans("torch_in_world", torch_in_world);
					
	TRANS t6_to_torch;
	t6_to_torch = Transform::getTrans6ToTorch();
	t6_to_torch.inverse();
				
	TRANS t6_in_world;
	t6_in_world = torch_in_world * t6_to_torch;
			
	TRANS w_to_base;
	w_to_base = Transform::getTransWorldToBase(ex_angle);
	w_to_base.inverse();
			
	TRANS t6_in_robot;
	t6_in_robot = w_to_base * t6_in_world;

//	print_trans("t6_in_robot", t6_in_robot);
	
	JAngle angle;
	if (rob->InverseRobot(angle, pre_angle, t6_in_robot)) {
#ifdef DEBUG
		printf("error inverserobot\n");
#endif
//		delete rob;
		return std::numeric_limits<double>::quiet_NaN();
	}

	// TRANS t01, t02, t03, t04, t05, t06;
	// Transform::getTransBaseToJoints(angle, t01, t02, t03, t04, t05, t06);

	// TRANS t6_in_robot1 = t01 * t02 * t03 * t04 * t05 * t06;

	// print_trans("t6_in_robot1", t6_in_robot1);

	TRANS t6;
	rob->PositiveRobot(angle, t6);

//	print_trans("t6", t6);

	TRANS torch_in_world1 = Transform::getTransWorldToBase(ex_angle) * t6 * Transform::getTrans6ToTorch();

//	print_trans("torch_in_world1", torch_in_world1);

	for (int i = 0; i < 6; i++)
		m_axes_values[i] = angle.get_angle(i + 1);
	
	// to_continuous(s->angle, pre_s->angle);
	// to_continuous(s->ex_angle, pre_s->ex_angle);
//	delete rob;


	
	m_auxiliary_variable_values[3] = calc_criteria(&angle)  / 376234706.2853961;

//	check();

	return criteria();
}

void kunshan_robot::print_trans(std::string name, TRANS& trans) {
	std::cout << std::endl << name << std::endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << trans.rot.mem[i][j] << " ";
		}
		std::cout << std::endl;
	}

	std::cout << trans.pos.dx << " " << trans.pos.dy << " " << trans.pos.dz << std::endl;
}

void kunshan_robot::check()
{
	JAngle angle;
	for (int i = 0; i < 6; i++) {
		angle.set_angle(m_axes_values[i], i + 1);
		std::cout << m_axes_values[i] << " ";
	}

	JAngle ex_angle(m_axes_values[6], m_axes_values[7], m_axes_values[8], 0.0, 0.0, 0.0);
	
	TRANS gun_in_seam(0.0, 0.0, -1.0, 
			  1.0, 0.0, 0.0, 
			  0.0, -1.0, 0.0,
			  0.0, 0.0, 0.0);

	TRANS rotateY(cos(m_auxiliary_variable_values[0]), 0.0, -sin(m_auxiliary_variable_values[0]),
		      0.0, 1.0, 0.0,
		      sin(m_auxiliary_variable_values[0]), 0.0, cos(m_auxiliary_variable_values[0]),
		      0.0, 0.0, 0.0);
	TRANS rotateX(1.0, 0.0, 0.0,
		      0.0, cos(m_auxiliary_variable_values[1]), sin(m_auxiliary_variable_values[1]),
		      0.0, -sin(m_auxiliary_variable_values[1]), cos(m_auxiliary_variable_values[1]),
		      0.0, 0.0, 0.0);

	TRANS rotateZ(cos(m_auxiliary_variable_values[2]), sin(m_auxiliary_variable_values[2]), 0.0,
		      -sin(m_auxiliary_variable_values[2]), cos(m_auxiliary_variable_values[2]), 0.0,
		      0.0, 0.0, 1.0,
		      0.0, 0.0, 0.0);
	gun_in_seam = rotateX * rotateY * gun_in_seam * rotateX;

	print_trans("gun_in_seam", gun_in_seam);

	Vector3D axis_y = m_n * m_t;
	TRANS seam_in_part(m_t.dx, m_t.dy, m_t.dz,
			   axis_y.dx, axis_y.dy, axis_y.dz,
			   m_n.dx, m_n.dy, m_n.dz,
			   m_p.dx, m_p.dy, m_p.dz);
	TRANS gun_in_part = seam_in_part * gun_in_seam;

	print_trans("gun_in_part", gun_in_part);

	TRANS part_trans;
	part_trans = Transform::getTransWorldToWorkpiece(ex_angle);

	TRANS torch_in_world;
	torch_in_world = part_trans * gun_in_part;

	print_trans("torch_in_world", torch_in_world);

	TRANS t6_in_base;
	rob->PositiveRobot(angle, t6_in_base);

	print_trans("t6_in_base", t6_in_base);
	// TRANS t01, t02, t03, t04, t05, t06;

	// Transform::getTransBaseToJoints(angle, t01, t02, t03, t04, t05, t06);

//	TRANS t6_in_base = t01 * t02 * t03 * t04 * t05 * t06;

	TRANS t6_to_torch;
	t6_to_torch = Transform::getTrans6ToTorch();

	print_trans("t6_to_torch", t6_to_torch);

	TRANS w_to_base;
	w_to_base = Transform::getTransWorldToBase(ex_angle);

	TRANS torch_in_world1 = w_to_base * t6_in_base * t6_to_torch;

	print_trans("torch_in_world1", torch_in_world1);
}
