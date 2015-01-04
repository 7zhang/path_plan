#include "KunShan_robot.h"
#include "state.h"
#include "Transform.h"
#include "calc_criteria.h"
#include "teach_point.h"


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

TRANS kunshan_robot::get_gun_in_seam()
{
	return TRANS(0.0, 0.0, -1.0, 
			  1.0, 0.0, 0.0, 
			  0.0, -1.0, 0.0,
			  0.0, 0.0, 0.0);
}

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
	return calc_criteria(&angle)  / 376234706.2853961;
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
