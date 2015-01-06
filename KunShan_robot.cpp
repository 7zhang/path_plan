#include "KunShan_robot.h"
#include "state.h"
#include "Transform.h"
#include "teach_point.h"
#include "det6x6.h"

/*
 * nine axes
 * six auxiliary variable: rpy and jacobi determinant
 * 
 *
 *
 */

void kunshan_robot::init(std::string& m_sys_name, 
			 int& m_redundancy,
			 int& m_axis_nr,
			 int& m_auxiliary_variable_nr,
			 std::vector<axis>& m_axes,
			 std::vector<axis>& m_auxiliary_variable,
			 std::vector<int>& m_map,
			 std::vector<teach_point>& m_teach_points,
			 std::vector<double>& m_weight)
{
	m_sys_name = "KunShan Robot System";
	m_redundancy = 6;
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
	m_auxiliary_variable[4] = axis(0.0, 15.0, 3.0, 3.0, 10, 0, 1.0);
	m_auxiliary_variable[5] = axis(75.0, 105.0, 3.0, 3.0, 10, 0, 1.0);

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

	mu_axis[0] = 36;
	sigma_axis[0] = 30;


	mu_axis[2] = 70;
	sigma_axis[2] = 30;
	
	mu_axis[7] = 150;
	sigma_axis[7] = 30;

	teach_point t1(mu_axis, sigma_axis, offset_axis, weight_axis, mu_axis, sigma_axis, offset_axis, weight_axis);
	m_teach_points.push_back(t1);
	m_weight.push_back(1.0);
}

TRANS kunshan_robot::get_gun_in_seam(const JAngle& weld_angle)
{
	TRANS gun_in_seam(0.0, 0.0, -1.0, 
			  1.0, 0.0, 0.0, 
			  0.0, -1.0, 0.0,
			  0.0, 0.0, 0.0);

	double pi = boost::math::constants::pi<double>();
	TRANS rotateY(cos(weld_angle.get_angle(1) / 180.0 * pi), 0.0, -sin(weld_angle.get_angle(1) / 180.0 * pi),
		      0.0, 1.0, 0.0,
		      sin(weld_angle.get_angle(1) / 180.0 * pi), 0.0, cos(weld_angle.get_angle(1) / 180.0 * pi),
		      0.0, 0.0, 0.0);
	TRANS rotateXL(1.0, 0.0, 0.0,
		       0.0, cos(weld_angle.get_angle(2) / 180.0 * pi), sin(weld_angle.get_angle(2) / 180.0 * pi),
		       0.0, -sin(weld_angle.get_angle(2) / 180.0 * pi), cos(weld_angle.get_angle(2) / 180.0 * pi),
		       0.0, 0.0, 0.0);

	TRANS rotateXR(1.0, 0.0, 0.0,
		       0.0, cos(weld_angle.get_angle(3) / 180.0 * pi), sin(weld_angle.get_angle(3) / 180.0 * pi),
		       0.0, -sin(weld_angle.get_angle(3) / 180.0 * pi), cos(weld_angle.get_angle(3) / 180.0 * pi),
		       0.0, 0.0, 0.0);

	// TRANS rotateZ(cos((*args)[5] / 180.0 * pi), sin((*args)[5] / 180.0 * pi), 0.0,
	// 	      -sin((*args)[5] / 180.0 * pi), cos((*args)[5] / 180.0 * pi), 0.0,
	// 	      0.0, 0.0, 1.0,
	// 	      0.0, 0.0, 0.0);
	gun_in_seam = rotateXL * rotateY * gun_in_seam * rotateXR;

	return gun_in_seam;
}

// TRANS kunshan_robot::get_gun_in_seam()
// {
// 	return TRANS(0.0, 0.0, -1.0, 
// 		     1.0, 0.0, 0.0, 
// 		     0.0, -1.0, 0.0,
// 		     0.0, 0.0, 0.0);
// }

TRANS kunshan_robot::getTransWorldToWorkpiece(JAngle ex_angle)
{
	return Transform::getTransWorldToWorkpiece(ex_angle);
}

TRANS kunshan_robot::getTrans6ToTorch()
{
	return Transform::getTrans6ToTorch();
}

TRANS kunshan_robot::getTransWorldToBase(JAngle ex_angle)
{
	return Transform::getTransWorldToBase(ex_angle);
}

bool kunshan_robot::InverseRobot(JAngle& Jointangle,const JAngle& lastJointangle,const TRANS& t6)
{
	return rob->InverseRobot(Jointangle, lastJointangle, t6);
}

double kunshan_robot::get_jacobi_deter(JAngle& angle)
{
	double j[6][6];
	jacobi(j, angle.get_angle(1), angle.get_angle(2), angle.get_angle(3), 
		angle.get_angle(4), angle.get_angle(5), angle.get_angle(6));
		
	return fabs(determinant(j)) / 376234706.2853961;
}

const double d1 = 729.41;
const double a2 = 306.03;
const double a3 = 614.0;
const double d4 = 524.0;
const double d5 = 82.0;
const double RADIANPERDEGREE = 3.1415926535897932 / 180;

void kunshan_robot::jacobi(double j[6][6], double theta1, double theta2, double theta3, double theta4, double theta5, double theta6)
{
	theta1 = theta1 * RADIANPERDEGREE;
	theta2 = theta2 * RADIANPERDEGREE;
	theta3 = theta3 * RADIANPERDEGREE;
	theta4 = theta4 * RADIANPERDEGREE;
	theta5 = theta5 * RADIANPERDEGREE;
	theta6 = theta6 * RADIANPERDEGREE;

	double s1, c1, s2, c2, s3, c3, s4, c4, s5, c5, s6, c6, s23, c23;
	s1 = sin(theta1);
	c1 = cos(theta1);
	s2 = sin(theta2);
	c2 = cos(theta2);	
	s3 = sin(theta3);
	c3 = cos(theta3);
	s4 = sin(theta4);
	c4 = cos(theta4);
	s5 = sin(theta5);
	c5 = cos(theta5);
	s6 = sin(theta6);
	c6 = cos(theta6);
	s23 = s2 * c3 + c2 * s3;
	c23 = c2 * c3 - s2 * s3;
	j[0][0] = -((a2 + a3*c2 + c23*(d4 + c5*d5))*s1) + d5*(c4*s1*s23 - c1*s4)*s5;
	j[0][1] = c1*(-(a3*s2) - (d4 + c5*d5)*s23 - c23*c4*d5*s5);
	j[0][2] = c1*(-((d4 + c5*d5)*s23) - c23*c4*d5*s5);
	j[0][3] = d5*(-(c4*s1) + c1*s23*s4)*s5;
	j[0][4] = -(d5*(c5*s1*s4 + c1*(c4*c5*s23 + c23*s5)));
	j[0][5] = 0;
	j[1][0] = c1*(a2 + a3*c2 + c23*(d4 + c5*d5)) - d5*(c1*c4*s23 + s1*s4)*s5;
	j[1][1] = s1*(-(a3*s2) - (d4 + c5*d5)*s23 - c23*c4*d5*s5);
	j[1][2] = s1*(-((d4 + c5*d5)*s23) - c23*c4*d5*s5);
	j[1][3] = d5*(c1*c4 + s1*s23*s4)*s5;
	j[1][4] = c1*c5*d5*s4 - d5*s1*(c4*c5*s23 + c23*s5);
	j[1][5] = 0;
	j[2][0] = 0;
	j[2][1] = -(a3*c2) - c23*(d4 + c5*d5) + c4*d5*s23*s5;
	j[2][2] = -(c23*(d4 + c5*d5)) + c4*d5*s23*s5;
	j[2][3] = c23*d5*s4*s5;
	j[2][4] = -(c23*c4*c5*d5) + d5*s23*s5;
	j[2][5] = 0;
	j[3][0] = 0;
	j[3][1] = -s1;
	j[3][2] = -s1;
	j[3][3] = c1*c23;
	j[3][4] = -(c4*s1) + c1*s23*s4;
	j[3][5] = c1*c23*c5 - (c1*c4*s23 + s1*s4)*s5;
	j[4][0] = 0;
	j[4][1] = c1;
	j[4][2] = c1;
	j[4][3] = c23*s1;
	j[4][4] = c1*c4 + s1*s23*s4;
	j[4][5] = c23*c5*s1 + (-(c4*s1*s23) + c1*s4)*s5;
	j[5][0] = 1;
	j[5][1] = 0;
	j[5][2] = 0;
	j[5][3] = -s23;
	j[5][4] = c23*s4;
	j[5][5] = -(c5*s23) - c23*c4*s5;
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
