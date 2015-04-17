#include "positioner.h"
#include <iostream>
#include <string>
#include <limits>

void print_trans(const std::string& trans_name, const TRANS& trans)
{
	std::cout << trans_name << std::endl;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			std::cout << trans.rot.mem[i][j] << '\t';
		}
		std::cout << std::endl;
	}
	std::cout << trans.pos.dx << '\t' << trans.pos.dy << '\t' << trans.pos.dz << '\t' << std::endl;
	std::cout << std::endl;
}

positioner::positioner()
{
	DHparameters DH1(0.0, 2151.0, 0.0);
	DHparameters DH2(0.0, -505.0, 90.0);
	DH.push_back(DH1);
	DH.push_back(DH2);
}

bool positioner::PositiveRobot(weld_point& weldpoint, const JAngle& JA)
{
	//已知焊缝在世界坐标系中的位姿，求焊缝倾角和焊缝转角
	TRANS world_to_base(-1.0, 0.0, 0.0,
			    0.0, 0.0, -1.0,
			    0.0, -1.0, 0.0,
			    2625, 1020, 1720);
	//print_trans("world_to_base", world_to_base);

	TRANS world_to_workpiece = world_to_base * DH[0].get_reftrans(JA.get_angle(1)) * DH[1].get_reftrans(JA.get_angle(2));
	//print_trans("world_to_workpiece", world_to_workpiece);

	/*Vector3D p(-1168.000000, 1088.000000, 804.000000);	
	  Vector3D n(-0.707107, -0.000000, 0.707107);	
	  Vector3D t(0.000000, -1.000000, 0.000000);
	  Vector3D y = n * t;
	  TRANS seam_in_part( t.dx, t.dy, t.dz,
	  y.dx, y.dy, y.dz,
	  n.dx, n.dy, n.dz,
	  p.dx, p.dy, p.dz);*/
	TRANS seam_in_part = weldpoint.get_trans();
	//print_trans("seam_in_part", seam_in_part);

	TRANS seam_in_world = world_to_workpiece * seam_in_part;
	//print_trans("seam_in_world", seam_in_world);

	RPY temp;
	seam_in_world.Trans2RPY(temp);
// 	std::cout << "RPY" << std::endl;
// 	std::cout << temp.orient.dx << '\t'  << temp.orient.dy << '\t'  << temp.orient.dz << '\t' << std::endl;	
// 	std::cout << temp.pos.dx<< '\t'  << temp.pos.dy << '\t'  << temp.pos.dz << '\t' << std::endl;	

	weldpoint.m_fi = temp.orient.dx;		//焊缝转角fi
	weldpoint.m_theta = temp.orient.dy;		//焊缝倾角theta
	return true;
}

int positioner::InverseRobot1(weld_point& weldpoint, vector<JAngle>& vecJointangle)
{
	double theta = weldpoint.m_theta * PI / 180.0;
	double fi = weldpoint.m_fi * PI / 180.0;

	TRANS world_to_base(-1.0, 0.0, 0.0,
			    0.0, 0.0, -1.0,
			    0.0, -1.0, 0.0,
			    2625, 1020, 1720);

	double w_p0_nx = world_to_base.rot.mem[0][0];
	double w_p0_ny = world_to_base.rot.mem[1][0];
	double w_p0_nz = world_to_base.rot.mem[2][0];

	double w_p0_ox = world_to_base.rot.mem[0][1];
	double w_p0_oy = world_to_base.rot.mem[1][1];
	double w_p0_oz = world_to_base.rot.mem[2][1];

	double w_p0_ax = world_to_base.rot.mem[0][2];
	double w_p0_ay = world_to_base.rot.mem[1][2];
	double w_p0_az = world_to_base.rot.mem[2][2];

	/*Vector3D point(-1168.000000, 1088.000000, 804.000000);	
	  Vector3D normal(-0.707107, -0.000000, 0.707107);	
	  Vector3D tangent(0.000000, -1.000000, 0.000000);
	  Vector3D y = normal * tangent;
	  TRANS seam_in_part( tangent.dx, tangent.dy, tangent.dz,
	  y.dx, y.dy, y.dz,
	  normal.dx, normal.dy, normal.dz,
	  point.dx, point.dy, point.dz);*/
	TRANS seam_in_part = weldpoint.get_trans();

	double p2_s_nx = seam_in_part.rot.mem[0][0];
	double p2_s_ny = seam_in_part.rot.mem[1][0];
	double p2_s_nz = seam_in_part.rot.mem[2][0];
	
	double p2_s_ox = seam_in_part.rot.mem[0][1];
	double p2_s_oy = seam_in_part.rot.mem[1][1];
	double p2_s_oz = seam_in_part.rot.mem[2][1];
	
	double p2_s_ax = seam_in_part.rot.mem[0][2];
	double p2_s_ay = seam_in_part.rot.mem[1][2];
	double p2_s_az = seam_in_part.rot.mem[2][2];

	double t0 = - p2_s_nx * sin(theta) + p2_s_ox * cos(theta) * sin(fi) + p2_s_ax * cos(theta) * cos(fi); 
	double v0 = - p2_s_ny * sin(theta) + p2_s_oy * cos(theta) * sin(fi) + p2_s_ay * cos(theta) * cos(fi); 
	double w0 = - p2_s_nz * sin(theta) + p2_s_oz * cos(theta) * sin(fi) + p2_s_az * cos(theta) * cos(fi); 

	double t = t0 / sqrt(t0 * t0 + v0 * v0 + w0 * w0);
	double v = v0 / sqrt(t0 * t0 + v0 * v0 + w0 * w0);
	double w = w0 / sqrt(t0 * t0 + v0 * v0 + w0 * w0);

	
	double delta = 0;
	double gama = 0;
	double theta1 = 0;
	double theta2 = 0;

	double cos_alph = cos(DH[1].alpha * PI / 180.0);
	double sin_alph = sin(DH[1].alpha * PI / 180.0);

	double a11 = 0;
	double a12 = 0;
	double a21 = 0;
	double a22 = 0;
	double cos_theta2 = 0;
	double sin_theta2 = 0;
	
	JAngle angle_temp1;
	JAngle angle_temp2;

	int ret = -1;

	if (!isZero(w_p0_nz))
	{
		delta = atan(w_p0_oz / w_p0_nz);
		double demonimator = w_p0_nz * sin_alph * w_p0_nz * sin_alph + w_p0_oz * sin_alph * w_p0_oz * sin_alph;
		if (isZero(demonimator))
		{
			cout << "w_p0_oz * sin_alph == 0" << endl;
			ret = -1;
			return ret;
		}
		gama = asin((w - w_p0_az * cos_alph) / sqrt(demonimator));
		////////////////////////////////////////////////////////
		angle_temp1.angle[0] = delta + gama;
		a11 = w_p0_nz * cos(angle_temp1.angle[0]) + w_p0_oz * sin(angle_temp1.angle[0]);
		a12 = - w_p0_nz * sin(angle_temp1.angle[0]) * cos_alph + w_p0_oz * cos(angle_temp1.angle[0]) * cos_alph + w_p0_az * sin_alph;
		a21 = a12;
		a22 = -a11;
		if (isZero(a11 * a22 - a21 * a12))
		{
//			cout << "(a11 * a22 - a21 * a12) == 0" << endl;
			if (isZero(t * a22 - v * a12)&&isZero(a11 * v - a21 * t))
			{
				//cout << "无穷解" << endl;
				cout << "theta1 =" << angle_temp1.angle[0] * 180 / PI << endl;
				cout << "theta = " << weldpoint.m_theta << ", fi = " << weldpoint.m_fi << endl;
				cout << "t = " << t << ", v = " << v << ", w = " << w << endl;
				cout << "demonimator = " << demonimator << endl;
//				return 1;
				ret = 1;
				angle_temp1.angle[1] = std::numeric_limits<double>::quiet_NaN();
				goto next1;
			}			
			else
			{
				//cout << "无解" << endl;	
//				return -1;
				ret = -1;
				return ret;
			}
		}
		cos_theta2 = (t * a22 - v * a12) / (a11 * a22 - a21 * a12);
		sin_theta2 = (a11 * v - a21 * t) / (a11 * a22 - a21 * a12);
		angle_temp1.angle[1] = atan2(sin_theta2, cos_theta2);
		////////////////////////////////////////////////////////
	next1:	if (gama >= 0)
		{
			angle_temp2.angle[0] = PI - gama + delta;
		} 
		else
		{
			angle_temp2.angle[0] =- PI - gama + delta;
		}
		
		a11 = w_p0_nz * cos(angle_temp2.angle[0]) + w_p0_oz * sin(angle_temp2.angle[0]);
		a12 = - w_p0_nz * sin(angle_temp2.angle[0]) * cos_alph + w_p0_oz * cos(angle_temp2.angle[0]) * cos_alph + w_p0_az * sin_alph;
		a21 = a12;
		a22 = -a11;
		if (isZero(a11 * a22 - a21 * a12))
		{
//			cout << "(a11 * a22 - a21 * a12) == 0" << endl;
			if (isZero(t * a22 - v * a12)&&isZero(a11 * v - a21 * t))
			{
				cout << "..." << endl;
				cout << "theta = " << weldpoint.m_theta << ", fi = " << weldpoint.m_fi << endl;
				cout << "t = " << t << ", v = " << v << ", w = " << w << endl;
				cout << "demonimator = " << demonimator << endl;
//				return 1;
				ret = 1;
				angle_temp2.angle[1] = std::numeric_limits<double>::quiet_NaN();
				goto end;
			}			
			else
			{
				//cout << "无解" << endl;	
//				return -1;
				ret = -1;
				return ret;
			}
		}
		cos_theta2 = (t * a22 - v * a12) / (a11 * a22 - a21 * a12);
		sin_theta2 = (a11 * v - a21 * t) / (a11 * a22 - a21 * a12);
		angle_temp2.angle[1] = atan2(sin_theta2, cos_theta2);
	}
	else
	{
		double demonimator = w_p0_oz * sin_alph;
		if (isZero(demonimator))
		{
			cout << "w_p0_oz * sin_alph == 0" << endl;
//			return -1;
			ret = -1;
			return ret;
		}
		////////////////////////////////////////////////////////
		angle_temp1.angle[0] = acos((w_p0_az * cos_alph - w) / (demonimator));
		a11 = w_p0_oz * sin(angle_temp1.angle[0]);
		a12 = w_p0_oz * cos(angle_temp1.angle[0]) * cos_alph + w_p0_az * sin_alph;
		a21 = a12;
		a22 = -a11;
		if (isZero(a11 * a22 - a21 * a12))
		{
//			cout << "(a11 * a22 - a21 * a12) == 0" << endl;
			if (isZero(t * a22 - v * a12)&&isZero(a11 * v - a21 * t))
			{
				//cout << "无穷解" << endl;
				cout << "theta1 =" << angle_temp1.angle[0] * 180 / PI << endl;
				cout << "theta = " << weldpoint.m_theta << ", fi = " << weldpoint.m_fi << endl;
				cout << "t = " << t << ", v = " << v << ", w = " << w << endl;
				cout << "demonimator = " << demonimator << endl;
//				return 1;
				ret = 1;
				angle_temp1.angle[1] = std::numeric_limits<double>::quiet_NaN();
				goto next2;
			}			
			else
			{
				//cout << "无解" << endl;	
//				return -1;
				ret = -1;
				return ret;
			}		
		}
		cos_theta2 = (t * a22 - v * a12) / (a11 * a22 - a21 * a12);
		sin_theta2 = (a11 * v - a21 * t) / (a11 * a22 - a21 * a12);
		angle_temp1.angle[1] = atan2(sin_theta2, cos_theta2);
		/////////////////////////////////////////////////////////
	next2:	angle_temp2.angle[0] = - acos((w_p0_az * cos_alph - w) / (w_p0_oz * sin_alph));
		a11 = w_p0_oz * sin(angle_temp2.angle[0]);
		a12 = w_p0_oz * cos(angle_temp2.angle[0]) * cos_alph + w_p0_az * sin_alph;
		a21 = a12;
		a22 = -a11;
		if (isZero(a11 * a22 - a21 * a12))
		{
//			cout << "(a11 * a22 - a21 * a12) == 0" << endl;
			if (isZero(t * a22 - v * a12)&&isZero(a11 * v - a21 * t))
			{
				cout << "......" << endl;
				cout << "theta = " << weldpoint.m_theta << ", fi = " << weldpoint.m_fi << endl;
				cout << "t = " << t << ", v = " << v << ", w = " << w << endl;
				cout << "demonimator = " << demonimator << endl;
//				return 1;
				ret = 1;
				angle_temp2.angle[1] = std::numeric_limits<double>::quiet_NaN();
				goto end;
			}			
			else
			{
				//cout << "无解" << endl;	
//				return -1;
				ret = -1;
				return ret;
			}	
		}
		cos_theta2 = (t * a22 - v * a12) / (a11 * a22 - a21 * a12);
		sin_theta2 = (a11 * v - a21 * t) / (a11 * a22 - a21 * a12);
		angle_temp2.angle[1] = atan2(sin_theta2, cos_theta2);
	}

	ret = 0;
end:	angle_temp1.set_angles(angle_temp1.angle[0] * 180 / PI, angle_temp1.angle[1] * 180 / PI, angle_temp1.angle[2] * 180 / PI, angle_temp1.angle[3] * 180 / PI, angle_temp1.angle[4] * 180 / PI, angle_temp1.angle[5] * 180 / PI);
	angle_temp2.set_angles(angle_temp2.angle[0] * 180 / PI, angle_temp2.angle[1] * 180 / PI, angle_temp2.angle[2] * 180 / PI, angle_temp2.angle[3] * 180 / PI, angle_temp2.angle[4] * 180 / PI, angle_temp2.angle[5] * 180 / PI);
	weldpoint.m_angle1 = angle_temp1;
	weldpoint.m_angle2 = angle_temp2;
	vecJointangle.push_back(angle_temp1);
	vecJointangle.push_back(angle_temp2);
	return ret;
}

int positioner::InverseRobot(weld_point& weldpoint, vector<JAngle>& vecJointangle)
{
	//已知焊缝倾角和焊缝转角，求变位机倾翻和旋转角度
	double theta = weldpoint.m_theta * PI / 180.0;
	double fi = weldpoint.m_fi * PI / 180.0;

	TRANS world_to_base(-1.0, 0.0, 0.0,
			    0.0, 0.0, -1.0,
			    0.0, -1.0, 0.0,
			    2625, 1020, 1720);

	double w_p0_nx = world_to_base.rot.mem[0][0];
	double w_p0_ny = world_to_base.rot.mem[1][0];
	double w_p0_nz = world_to_base.rot.mem[2][0];

	double w_p0_ox = world_to_base.rot.mem[0][1];
	double w_p0_oy = world_to_base.rot.mem[1][1];
	double w_p0_oz = world_to_base.rot.mem[2][1];

	double w_p0_ax = world_to_base.rot.mem[0][2];
	double w_p0_ay = world_to_base.rot.mem[1][2];
	double w_p0_az = world_to_base.rot.mem[2][2];

	/*Vector3D point(-1168.000000, 1088.000000, 804.000000);	
	  Vector3D normal(-0.707107, -0.000000, 0.707107);	
	  Vector3D tangent(0.000000, -1.000000, 0.000000);
	  Vector3D y = normal * tangent;
	  TRANS seam_in_part( tangent.dx, tangent.dy, tangent.dz,
	  y.dx, y.dy, y.dz,
	  normal.dx, normal.dy, normal.dz,
	  point.dx, point.dy, point.dz);*/
	TRANS seam_in_part = weldpoint.get_trans();

	double p2_s_nx = seam_in_part.rot.mem[0][0];
	double p2_s_ny = seam_in_part.rot.mem[1][0];
	double p2_s_nz = seam_in_part.rot.mem[2][0];
	
	double p2_s_ox = seam_in_part.rot.mem[0][1];
	double p2_s_oy = seam_in_part.rot.mem[1][1];
	double p2_s_oz = seam_in_part.rot.mem[2][1];
	
	double p2_s_ax = seam_in_part.rot.mem[0][2];
	double p2_s_ay = seam_in_part.rot.mem[1][2];
	double p2_s_az = seam_in_part.rot.mem[2][2];

	double t0 = - p2_s_nx * sin(theta) + p2_s_ox * cos(theta) * sin(fi) + p2_s_ax * cos(theta) * cos(fi); 
	double v0 = - p2_s_ny * sin(theta) + p2_s_oy * cos(theta) * sin(fi) + p2_s_ay * cos(theta) * cos(fi); 
	double w0 = - p2_s_nz * sin(theta) + p2_s_oz * cos(theta) * sin(fi) + p2_s_az * cos(theta) * cos(fi); 

	double t = t0 / sqrt(t0 * t0 + v0 * v0 + w0 * w0);
	double v = v0 / sqrt(t0 * t0 + v0 * v0 + w0 * w0);
	double w = w0 / sqrt(t0 * t0 + v0 * v0 + w0 * w0);

	
	double delta = 0;
	double gama = 0;
	double theta1 = 0;
	double theta2 = 0;

	double cos_alph = cos(DH[1].alpha * PI / 180.0);
	double sin_alph = sin(DH[1].alpha * PI / 180.0);

	double a11 = 0;
	double a12 = 0;
	double a21 = 0;
	double a22 = 0;
	double cos_theta2 = 0;
	double sin_theta2 = 0;
	
	JAngle angle_temp1;
	JAngle angle_temp2;

	if (!isZero(w_p0_nz))
	{
		delta = atan(w_p0_oz / w_p0_nz);
		if (isZero(w_p0_nz * sin_alph * w_p0_nz * sin_alph + w_p0_oz * sin_alph * w_p0_oz * sin_alph))
		{
			return -1;
		}
		gama = asin((w - w_p0_az * cos_alph) / sqrt(w_p0_nz * sin_alph * w_p0_nz * sin_alph + w_p0_oz * sin_alph * w_p0_oz * sin_alph));
		////////////////////////////////////////////////////////
		angle_temp1.angle[0] = delta + gama;
		a11 = w_p0_nz * cos(angle_temp1.angle[0]) + w_p0_oz * sin(angle_temp1.angle[0]);
		a12 = - w_p0_nz * sin(angle_temp1.angle[0]) * cos_alph + w_p0_oz * cos(angle_temp1.angle[0]) * cos_alph + w_p0_az * sin_alph;
		a21 = a12;
		a22 = -a11;
		if (isZero(a11 * a22 - a21 * a12))
		{
//			cout << "(a11 * a22 - a21 * a12) == 0" << endl;
			if (isZero(t * a22 - v * a12)&&isZero(a11 * v - a21 * t))
			{
				//cout << "无穷解" << endl;
				cout << "theta1 =" << angle_temp1.angle[0] * 180 / PI << endl;
				return 1;
			}			
			else
			{
				//cout << "无解" << endl;	
				return -1;
			}
		}
		cos_theta2 = (t * a22 - v * a12) / (a11 * a22 - a21 * a12);
		sin_theta2 = (a11 * v - a21 * t) / (a11 * a22 - a21 * a12);
		angle_temp1.angle[1] = atan2(sin_theta2, cos_theta2);
		////////////////////////////////////////////////////////
		if (gama >= 0)
		{
			angle_temp2.angle[0] = PI - gama + delta;
		} 
		else
		{
			angle_temp2.angle[0] =- PI - gama + delta;
		}
		
		a11 = w_p0_nz * cos(angle_temp2.angle[0]) + w_p0_oz * sin(angle_temp2.angle[0]);
		a12 = - w_p0_nz * sin(angle_temp2.angle[0]) * cos_alph + w_p0_oz * cos(angle_temp2.angle[0]) * cos_alph + w_p0_az * sin_alph;
		a21 = a12;
		a22 = -a11;
		if (isZero(a11 * a22 - a21 * a12))
		{
//			cout << "(a11 * a22 - a21 * a12) == 0" << endl;
			if (isZero(t * a22 - v * a12)&&isZero(a11 * v - a21 * t))
			{
				cout << "..." << endl;
				return 1;
			}			
			else
			{
				//cout << "无解" << endl;	
				return -1;
			}
		}
		cos_theta2 = (t * a22 - v * a12) / (a11 * a22 - a21 * a12);
		sin_theta2 = (a11 * v - a21 * t) / (a11 * a22 - a21 * a12);
		angle_temp2.angle[1] = atan2(sin_theta2, cos_theta2);
	}
	else
	{
		if (isZero(w_p0_oz * sin_alph))
		{
//			cout << "w_p0_oz * sin_alph == 0" << endl;
			return -1;
		}
		////////////////////////////////////////////////////////
		angle_temp1.angle[0] = acos((w_p0_az * cos_alph - w) / (w_p0_oz * sin_alph));
		a11 = w_p0_oz * sin(angle_temp1.angle[0]);
		a12 = w_p0_oz * cos(angle_temp1.angle[0]) * cos_alph + w_p0_az * sin_alph;
		a21 = a12;
		a22 = -a11;
		if (isZero(a11 * a22 - a21 * a12))
		{
//			cout << "(a11 * a22 - a21 * a12) == 0" << endl;
			if (isZero(t * a22 - v * a12)&&isZero(a11 * v - a21 * t))
			{
				//cout << "无穷解" << endl;
				cout << "theta1 =" << angle_temp1.angle[0] * 180 / PI << endl;
				return 1;
			}			
			else
			{
				//cout << "无解" << endl;	
				return -1;
			}		
		}
		cos_theta2 = (t * a22 - v * a12) / (a11 * a22 - a21 * a12);
		sin_theta2 = (a11 * v - a21 * t) / (a11 * a22 - a21 * a12);
		angle_temp1.angle[1] = atan2(sin_theta2, cos_theta2);
		/////////////////////////////////////////////////////////
		angle_temp2.angle[0] = - acos((w_p0_az * cos_alph - w) / (w_p0_oz * sin_alph));
		a11 = w_p0_oz * sin(angle_temp2.angle[0]);
		a12 = w_p0_oz * cos(angle_temp2.angle[0]) * cos_alph + w_p0_az * sin_alph;
		a21 = a12;
		a22 = -a11;
		if (isZero(a11 * a22 - a21 * a12))
		{
//			cout << "(a11 * a22 - a21 * a12) == 0" << endl;
			if (isZero(t * a22 - v * a12)&&isZero(a11 * v - a21 * t))
			{
				cout << "......" << endl;
				return 1;
			}			
			else
			{
				//cout << "无解" << endl;	
				return -1;
			}	
		}
		cos_theta2 = (t * a22 - v * a12) / (a11 * a22 - a21 * a12);
		sin_theta2 = (a11 * v - a21 * t) / (a11 * a22 - a21 * a12);
		angle_temp2.angle[1] = atan2(sin_theta2, cos_theta2);
	}
	angle_temp1.set_angles(angle_temp1.angle[0] * 180 / PI, angle_temp1.angle[1] * 180 / PI, angle_temp1.angle[2] * 180 / PI, angle_temp1.angle[3] * 180 / PI, angle_temp1.angle[4] * 180 / PI, angle_temp1.angle[5] * 180 / PI);
	angle_temp2.set_angles(angle_temp2.angle[0] * 180 / PI, angle_temp2.angle[1] * 180 / PI, angle_temp2.angle[2] * 180 / PI, angle_temp2.angle[3] * 180 / PI, angle_temp2.angle[4] * 180 / PI, angle_temp2.angle[5] * 180 / PI);
	weldpoint.m_angle1 = angle_temp1;
	weldpoint.m_angle2 = angle_temp2;
	vecJointangle.push_back(angle_temp1);
	vecJointangle.push_back(angle_temp2);
	return 0;
}

// bool positioner::InverseRobotEx(vector<JAngle>& vecJointangle, const TRANS& t6)
// {
// 	return true;
// }
