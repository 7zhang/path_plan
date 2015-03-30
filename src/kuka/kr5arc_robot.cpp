#include "kuka/kr5arc_robot.h"
#include "teach_point.h"
#include "det6x6.h"
#include "cd.h"

/*
 * ten axes
 * six auxiliary variable: rpy and jacobi determinant
 * 
 *
 *
 */
KR5ARC_RKA kr5;
//int output = 0;

void KR5ARC_robot::init(std::string& m_sys_name, 
			int& m_redundancy,
			int& m_axis_nr,
			int& m_auxiliary_variable_nr,
			std::vector<axis>& m_axes,
			std::vector<axis>& m_auxiliary_variable,
			std::vector<int>& m_map,
			std::vector<teach_point>& m_teach_points,
			std::vector<double>& m_weight, std::vector<double>& para)
{
	m_sys_name = "KR5ARC Robot System";	//modified
	m_redundancy = 7;			//modified
	m_axis_nr = 10;				//modified
	m_axes.resize(10);			//modified
	//robot's joint  angles
	m_axes[0] = axis(-155.0, 155.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[1] = axis(-180.0, 65.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[2] = axis(-15.0, 158.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[3] = axis(-350.0, 350.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[4] = axis(-130.0, 130.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[5] = axis(-350.0, 350.0, 50.0, 10.0, 10, 0, 1.0);
	//positioner's joint angles
	m_axes[6] = axis(-185.0, 185.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[7] = axis(-360.0, 360.0, 50.0, 10.0, 10, 0, 1);
	//C-style brace's joint angles
	m_axes[8] = axis(-180.0, 180.0, 50.0, 10.0, 10, 0, 1.0);
	m_axes[9] = axis(-750.0, 750.0, 50.0, 10.0, 10, 0, 1.0);		//modified

	m_auxiliary_variable_nr = 6;
	m_auxiliary_variable.resize(6);
	m_auxiliary_variable[0] = axis(-30.0, 0.0, 3.0, 3.0, 10, 0, 1.0);  	//gun's work angle
	m_auxiliary_variable[1] = axis(-15.0, 15.0, 3.0, 3.0, 10, 0, 1.0); 	//gun's walking angle
	m_auxiliary_variable[2] = axis(-180.0, 180.0, 3.0, 3.0, 10, 0, 1.0);	//gun's rotation angle
	m_auxiliary_variable[3] = axis(0.0, 1.0, 3.0, 3.0, 10, 3, 1.0);		//Jacobi matrix determinant
	m_auxiliary_variable[4] = axis(-15.0 + para[0], 15.0 + para[0], 3.0, 3.0, 10, 0, 1.0);	//weld slope angle
	m_auxiliary_variable[5] = axis(-15.0 + para[1], 15.0 + para[1], 3.0, 3.0, 10, 0, 3.0);	//weld rotation angle

	std::cout << "para: " << para[0] << " " << para[1] << std::endl;
	m_map.push_back(6);
	m_map.push_back(7);
	m_map.push_back(8);
	m_map.push_back(9);
	m_map.push_back(10);
	m_map.push_back(11);
	m_map.push_back(12);				//modified

	std::vector<double> mu_axis(10);		//modified
	std::vector<double> sigma_axis(10);		//modified
	std::vector<double> offset_axis(10, 0);		//modified
	std::vector<double> weight_axis(10, 1.0);	//modified
	
	for (int i = 0; i < m_axes.size(); i++) {
		mu_axis[i] = m_axes[i].mid();
		sigma_axis[i] = m_axes[i].length() / 6.0;
	}
        // mu_axis[0] = -0.397979;
        // sigma_axis[0] = 20;

        // mu_axis[1] = 34.0548;
        // sigma_axis[1] = 20;

        // mu_axis[2] = 56.86620;
        // sigma_axis[2] = 20;

        // mu_axis[3] = -31.8202;
        // sigma_axis[3] = 20;

        // mu_axis[4] = -70.2386;
        // sigma_axis[4] = 20;

        // mu_axis[5] = -145.726;
        // sigma_axis[5] = 20;

        // mu_axis[6] = 53.6836;
        // sigma_axis[6] = 20;
        
        // mu_axis[7] = -91.7367;
        // sigma_axis[7] = 20;

        // mu_axis[8] = 55.95020;
        // sigma_axis[8] = 20;

        // mu_axis[9] = 440.602;
        // sigma_axis[9] = 20;

	// std::vector<double> aux_axis(6);
	// std::vector<double> aux_sigma_axis(6);
	// aux_axis[0] = -13.9378;
        // aux_sigma_axis[0] = 20;

        // aux_axis[1] = -3.91739;
        // aux_sigma_axis[1] = 20;

        // aux_axis[2] = 2.11313;
        // aux_sigma_axis[2] = 20;

        // aux_axis[3] = 0.550153;
        // aux_sigma_axis[3] = 20;

        // aux_axis[4] = 8.84606;
        // aux_sigma_axis[4] = 20;

        // aux_axis[5] = 88.6357;
        // aux_sigma_axis[5] = 20;

	
	// // mu_axis[0] = 36;
	// // sigma_axis[0] = 30;


	// // mu_axis[2] = 70;
	// // sigma_axis[2] = 30;
	
	// // mu_axis[7] = 150;
	// // sigma_axis[7] = 30;

	

	// teach_point t1(mu_axis, sigma_axis, offset_axis, weight_axis, aux_axis, aux_sigma_axis, offset_axis, weight_axis);
	// m_teach_points.push_back(t1);
//	m_weight.push_back(1.0);

	cd_initialize();
}
/***************************************************************************/
double KR5ARC_robot::operator() (de::DVectorPtr args) {
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

	JAngle gun_angle((*args)[4], (*args)[5], (*args)[6], 0.0, 0.0, 0.0);
	TRANS gun_in_seam = this->get_gun_in_seam(gun_angle);
	//TRANS gun_in_seam(0.0, 1.0, 0.0, 
	//		  1.0, 0.0, 0.0, 
	//		  0.0, 0.0, -1.0,
	//		  0.0, 0.0, 0.0);				//modified
	

//	print_trans("gun_in_seam", gun_in_seam);
	
	Vector3D axis_y = m_n * m_t;
	TRANS seam_in_part(m_t.dx, m_t.dy, m_t.dz,
			   axis_y.dx, axis_y.dy, axis_y.dz,
			   m_n.dx, m_n.dy, m_n.dz,
			   m_p.dx, m_p.dy, m_p.dz);
//	print_trans("seam_in_part", seam_in_part);
	TRANS gun_in_part = seam_in_part * gun_in_seam;

//	print_trans("gun_in_part", gun_in_part);

	JAngle ex_angle((*args)[0], (*args)[1], (*args)[2], (*args)[3], 0.0, 0.0);//modified
	TRANS part_trans;
	
	part_trans = kr5.getTransWorldToWorkpiece(m_pos, 3, ex_angle);

//	print_trans("part_trans", part_trans);

	Vector3D n1, t1;
	Vector3D up(0.0, 0.0, 1.0);
	n1 = part_trans * m_n;
	t1 = part_trans * m_t;
	m_auxiliary_variable_values[4] = 90 - angle_between(t1, up);
	double d = sqrt(t1.dx * t1.dx + t1.dy * t1.dy);
	if (d < 1e-6) {
		m_auxiliary_variable_values[5] = (m_auxiliary_variable[5].min() + m_auxiliary_variable[5].max()) / 2;
		std::cout << "d < 1e-6" << std::endl;
	} else {
		Vector3D gb_16672_yaxis(t1.dy / d, - t1.dx / d, 0.0);
		axis_y = part_trans * axis_y;
		double tmp = gb_16672_yaxis ^ axis_y;
		if (tmp < 0.0) {
			gb_16672_yaxis = gb_16672_yaxis * (-1.0);
		}
		if (n1.dz > 0.0) {
			m_auxiliary_variable_values[5] = angle_between(n1, gb_16672_yaxis);
		} else {
			m_auxiliary_variable_values[5] = -1.0 * angle_between(n1, gb_16672_yaxis);
		}
	}
				
	TRANS torch_in_world;
	torch_in_world = part_trans * gun_in_part;

//	print_trans("torch_in_world", torch_in_world);
					
	TRANS t6_to_torch;
	t6_to_torch = this->getTrans6ToTorch();
//	print_trans("t6_to_torch", t6_to_torch);
	t6_to_torch.inverse();
				
	TRANS t6_in_world;
	t6_in_world = torch_in_world * t6_to_torch;
//	print_trans("t6_in_world", t6_in_world);
			
	TRANS w_to_base;
	w_to_base = this->getTransWorldToBase(ex_angle);
//	print_trans("w_to_base", w_to_base);
	w_to_base.inverse();
			
	TRANS t6_in_robot;
	t6_in_robot = w_to_base * t6_in_world;

//	print_trans("t6_in_robot", t6_in_robot);
	
	JAngle angle;
	if (InverseRobot(angle, pre_angle, t6_in_robot)) {
#ifdef DEBUG
		printf("error inverserobot\n");
#endif
//		delete rob;
		m_cri = std::numeric_limits<double>::quiet_NaN();
		return m_cri;
	}

	// TRANS t01, t02, t03, t04, t05, t06;
	// Transform::getTransBaseToJoints(angle, t01, t02, t03, t04, t05, t06);

	// TRANS t6_in_robot1 = t01 * t02 * t03 * t04 * t05 * t06;

//	std::cout << "angle" << std::endl;
//	for (int i = 0; i < 6; i++) {
//		std::cout << angle.get_angle(i + 1) << std::endl;	
//	}	
// 	TRANS t6_in_robot1;
// 	rob->PositiveRobot(angle, t6_in_robot1);
//
// 	print_trans("t6_in_robot1", t6_in_robot1);
//
// 	TRANS t6;
// 	rob->PositiveRobot(angle, t6);
//
//	print_trans("t6", t6);
//
// 	TRANS world_to_base1 = this->getTransWorldToBase(ex_angle);
// 	TRANS t6_to_torch1 = this->getTrans6ToTorch();
// 	TRANS torch_in_world1 = world_to_base1 * t6_in_robot1 * t6_to_torch1;
//
// 	print_trans("world_to_base1", world_to_base1);
// 	print_trans("t6_to_torch1", t6_to_torch1);
// 	print_trans("torch_in_world1", torch_in_world1);

	for (int i = 0; i < 6; i++)
		m_axes_values[i] = angle.get_angle(i + 1);
	
	// to_continuous(s->angle, pre_s->angle);
	// to_continuous(s->ex_angle, pre_s->ex_angle);
//	delete rob;

	if (cd_detect && cd()) {
		m_cri = std::numeric_limits<double>::quiet_NaN();
		return m_cri;
	}
	
	m_auxiliary_variable_values[3] = get_jacobi_deter(angle);
//calc_criteria(&angle)  / 376234706.2853961;

//	check();

	return criteria();
}

std::vector<volumenode *> KR5ARC_robot::left_node;
std::vector<volumenode *> KR5ARC_robot::right_node;

KR5ARC_robot::KR5ARC_robot(int axis_nr, int auxiliary_variable_nr,
			   const int pos,
			   const std::vector<double> para, 
			   const Vector3D& p, const Vector3D& n, 
			   const Vector3D& t, const std::vector<axis>& axes, 
			   const std::vector<axis>& auxiliary_variable, const std::vector<int>& map,
			   const std::vector<teach_point>& teach_points, const std::vector<double>& weight)
	: system_state(axis_nr,auxiliary_variable_nr, pos, para, p, n, t,
		       axes, auxiliary_variable, map, teach_points, weight)
{
	rob = new KR5ARC_RKA(); 


}

void KR5ARC_robot::cd_initialize()
{
	if (left_node.size() != 0) {
		return;
	}
	cd_parameter cd_para;
	cd_para.max_length = 60;
	cd_para.max_triangle = 5;

	std::vector<std::string> left_path, right_path;
	left_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_lbase.STL");
	left_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_larm.STL");
	left_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_plate.STL");
	left_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_workpiece.STL");

	right_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_rotate.STL");
	right_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_updown.STL");
	right_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_1.STL");
	right_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_2.STL");
	right_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_3.STL");
	right_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_4.STL");
	right_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_5.STL");
	right_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_6.STL");
	right_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_7.STL");
	right_path.push_back("/home/zhang7/path_plan/cd/robot_stl/kr16_gun.STL");

	for (int i = 0; i < left_path.size(); i++) {
		volumenode *ret = cd_init(left_path[i].c_str(), &cd_para);
		if (ret == NULL)
		{
			std::cout << "cd_init error, STL: " << left_path[i] << std::endl;
			goto error;
		}
		
		left_node.push_back(ret);
	}

	for (int i = 0; i < right_path.size(); i++) {
		volumenode *ret = cd_init(right_path[i].c_str(), &cd_para);
		if (ret == NULL)
		{
			std::cout << "cd_init error, STL: " << right_path[i] << std::endl;
			goto error;
		}
		
		right_node.push_back(ret);
	}

	return;

error:
	for (int i = 0; i < left_node.size(); i++) {
		cd_finish(left_node[i]);
	}

	for (int i = 0; i < right_node.size(); i++) {
		cd_finish(right_node[i]);
	}

	return;
}

int KR5ARC_robot::cd()
{
	JAngle angle(m_axes_values[0], m_axes_values[1], m_axes_values[2],
				  m_axes_values[3], m_axes_values[4], m_axes_values[5]);
	JAngle ex_angle(m_axes_values[6], m_axes_values[7], m_axes_values[8], m_axes_values[9], 0.0, 0.0);
//	KR5ARC_RKA kr5;
	for (int i = 0; i < left_node.size(); i++) {
		for (int j = 0; j < right_node.size(); j++) {					
			TRANS left_trans = kr5.getTransWorldToWorkpiece(m_pos, i, ex_angle);
//			log_trans("left_trans", left_trans);
			TRANS right_trans = kr5.get_trans_to_world(j, angle, ex_angle);
//			log_trans("right_trans", right_trans);

			vector3d v1, v2;
			v1.x = left_trans.pos.dx;
			v1.y = left_trans.pos.dy;
			v1.z = left_trans.pos.dz;

			v2.x = right_trans.pos.dx;
			v2.y = right_trans.pos.dy;
			v2.z = right_trans.pos.dz;

			int result = collision_detection2(left_node[i], left_trans.rot.mem, 
							  &v1, right_node[j], right_trans.rot.mem, &v2);
			if (result) {
				return result;
			}
		}
	}

	return 0;
}
void KR5ARC_robot::print_trans(std::string name, TRANS& trans) {
	std::cout << std::endl << name << std::endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << trans.rot.mem[i][j] << " ";
		}
		std::cout << std::endl;
	}

	std::cout << trans.pos.dx << " " << trans.pos.dy << " " << trans.pos.dz << std::endl;
}

bool KR5ARC_robot::InverseRobot(JAngle& Jointangle,const JAngle& lastJointangle,const TRANS& t6)
{
	return rob->InverseRobot(Jointangle, lastJointangle, t6);
}

double KR5ARC_robot::get_jacobi_deter(JAngle& angle)
{
	double j[6][6];
	jacobi(j, angle.get_angle(1), angle.get_angle(2), angle.get_angle(3), 
	       angle.get_angle(4), angle.get_angle(5), angle.get_angle(6));
		
	return fabs(determinant(j)) / 587233000;
}

const double d1 = 675;
const double a1 = 260;
const double a2 = 680;
const double a3 = 35;
const double d4 = -670.0;
const double RADIANPERDEGREE = 3.1415926535897932 / 180;

void KR5ARC_robot::jacobi(double j[6][6], double theta1, double theta2, double theta3, double theta4, double theta5, double theta6)
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
	j[0][0] = -(s1*(a1 + a2*c2 + a3*c23 - d4*s23));
	j[0][1] = -(c1*(c23*d4 + a2*s2 + a3*s23));
	j[0][2] = -(c1*(c23*d4 + a3*s23));
	j[0][3] = 0.0;
	j[0][4] = 0.0;
	j[0][5] = 0.0;

	j[1][0] = c1*(a1 + a2*c2 + a3*c23 - d4*s23);
	j[1][1] = -(s1*(c23*d4 + a2*s2 + a3*s23));
	j[1][2] = -(s1*(c23*d4 + a3*s23));
	j[1][3] = 0.0;
	j[1][4] = 0.0;
	j[1][5] = 0.0;

	j[2][0] = 0.0;
	j[2][1] = -(a2*c2) - a3*c23 + d4*s23;
	j[2][2] = -(a3*c23) + d4*s23;
	j[2][3] = 0.0;
	j[2][4] = 0.0;
	j[2][5] = 0.0;

	j[3][0] = 0.0;
	j[3][1] = -s1;
	j[3][2] = -s1;
	j[3][3] = -(c1*s23);
	j[3][4] = -(c4*s1) + c1*c23*s4;
	j[3][5] = -(s1*s4*s5) - c1*(c5*s23 + c23*c4*s5);

	j[4][0] = 0.0;
	j[4][1] = c1;
	j[4][2] = c1;
	j[4][3] = -(s1*s23);
	j[4][4] = c1*c4 + c23*s1*s4;
	j[4][5] = -(c5*s1*s23) + (-(c23*c4*s1) + c1*s4)*s5;

	j[5][0] = 1;
	j[5][1] = 0.0;
	j[5][2] = 0.0;
	j[5][3] = -c23;
	j[5][4] = -(s23*s4);
	j[5][5] = -(c23*c5) + c4*s23*s5;
}

/***************************************************************************/

/*
 * transformation 
 *
 */
TRANS KR5ARC_robot::RotateX(double angle)
{
	TRANS res;
	res.rotx(angle);
	return res;
}

TRANS KR5ARC_robot::RotateY(double angle)
{
	TRANS res;
	res.roty(angle);
	return res;
}

TRANS KR5ARC_robot::RotateZ(double angle)
{
	TRANS res;
	res.rotz(angle);
	return res;
}

TRANS KR5ARC_robot::Trans(double x, double y, double z)
{
	TRANS res;
	res.trans(x, y, z);
	return res;
}

TRANS KR5ARC_robot::get_gun_in_seam(const JAngle& weld_angle)
{
	double pi = boost::math::constants::pi<double>();
	TRANS gun_in_seam(0.0, 1.0, 0.0, 
			  1.0, 0.0, 0.0, 
			  0.0, 0.0, -1.0,
			  0.0, 0.0, 0.0);
	
	TRANS rotateY(cos(weld_angle.get_angle(1) / 180.0 * pi), 0.0, -sin(weld_angle.get_angle(1) / 180.0 * pi),
		      0.0, 1.0, 0.0,
		      sin(weld_angle.get_angle(1) / 180.0 * pi), 0.0, cos(weld_angle.get_angle(1) / 180.0 * pi),
		      0.0, 0.0, 0.0);					//modified
	TRANS rotateX(1.0, 0.0, 0.0,
		      0.0, cos(weld_angle.get_angle(2) / 180.0 * pi), sin(weld_angle.get_angle(2) / 180.0 * pi),
		      0.0, -sin(weld_angle.get_angle(2) / 180.0 * pi), cos(weld_angle.get_angle(2) / 180.0 * pi),
		      0.0, 0.0, 0.0);					//modified
		
	//TRANS rotateZ(cos((*args)[5] / 180.0 * pi), sin((*args)[5] / 180.0 * pi), 0.0,
	//	      -sin((*args)[5] / 180.0 * pi), cos((*args)[5] / 180.0 * pi), 0.0,
	//	      0.0, 0.0, 1.0,
	//	      0.0, 0.0, 0.0);
	TRANS rotateZ(cos(weld_angle.get_angle(3) / 180.0 * pi), sin(weld_angle.get_angle(3) / 180.0 * pi), 0.0,
		      -sin(weld_angle.get_angle(3) / 180.0 * pi), cos(weld_angle.get_angle(3) / 180.0 * pi), 0.0,
	 	      0.0, 0.0, 1.0,
		      0.0, 0.0, 0.0);					//modified
	gun_in_seam = rotateX * rotateY * gun_in_seam * rotateZ;	//modified
	return gun_in_seam;
}

TRANS KR5ARC_robot::getTransWorldToWorkpiece(JAngle ex_angle)
{
	double ext1 = ex_angle.angle[0];
	double ext2 = ex_angle.angle[1];
	TRANS TP1 = Trans(2625.0, 1020.0, 1720.0)*RotateY(90)*RotateX(90)*Trans(0, 0, 106)*RotateZ(-90)*RotateZ(ext1);
	TRANS TP2 = TP1*Trans(0.0,505.0,2045.0)*RotateX(90)*RotateZ(ext2);
	//return TP2*RotateZ(90);
	return TP2;
}

TRANS KR5ARC_robot::getTrans6ToTorch()
{
	TRANS T6ToFlange(0.0, 0.0, -1.0, 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -158.0);
	TRANS flangleToEnd = Trans(457.64, 0.0, 54.85)*RotateY(112.0);
	return T6ToFlange*flangleToEnd;
}

TRANS KR5ARC_robot::getTransWorldToBase(JAngle ex_angle)
{
	TRANS C0=Trans(0, -675.0, 0)*RotateZ(-90)*RotateX(90); 
	TRANS C1=Trans(0, 176.0, 0)*RotateY(90)*RotateX(-90)*RotateZ(ex_angle.angle[2]);
	TRANS C2=Trans(0, -388.0, 2822.0)*RotateX(90)*Trans(0, ex_angle.angle[3], 0);
	return C0*C1*C2*Trans(0.0, 78.0, 1323.0)*RotateY(-90)*RotateX(90);	
}
/****************************************************************************/
