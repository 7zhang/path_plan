#define __ROBOTDATA_EXP_
#include "robotdata.h"
#include "kuka/kr5arc.h"
#include <cassert>
#include <math.h>

//**************KR5ARC.cpp**********************
// Method:    PositiveRobot
// FullName:  PositiveRobot
// Access:    public 
// Returns:   extern "C"
// Qualifier: UINT KR5ARC_PositiveRobot(const JAngle& JA,TRANS& t6)
// Parameter: KR5ARC机器人正解
// Time:      2014/12/23
// Author:	  杨明亮
// 东南大学智能机器人实验室
//************************************

bool KR5ARC_PositiveRobot(const JAngle& JA,TRANS& t6)
{
	TRANS temp;
	double s1=sin(JA.angle[0]*PI/180.0);
	double c1=cos(JA.angle[0]*PI/180.0);
	double s2=sin(JA.angle[1]*PI/180.0);
	double c2=cos(JA.angle[1]*PI/180.0);
	double s3=sin(JA.angle[2]*PI/180.0);
	double c3=cos(JA.angle[2]*PI/180.0);
	double s4=sin(JA.angle[3]*PI/180.0);
	double c4=cos(JA.angle[3]*PI/180.0);
	double s5=sin(JA.angle[4]*PI/180.0);
	double c5=cos(JA.angle[4]*PI/180.0);
	double s6=sin(JA.angle[5]*PI/180.0);
	double c6=cos(JA.angle[5]*PI/180.0);     

	temp.rot.mem[0][0] = s6*(c4*s1 + s4*(c1*s2*s3 - c1*c2*c3)) + c6*(c5*(s1*s4 - c4*(c1*s2*s3 - c1*c2*c3)) - s5*(c1*c2*s3 + c1*c3*s2));
	temp.rot.mem[1][0] = - s6*(c1*c4 - s4*(s1*s2*s3 - c2*c3*s1)) - c6*(c5*(c1*s4 + c4*(s1*s2*s3 - c2*c3*s1)) + s5*(c2*s1*s3 + c3*s1*s2));
	temp.rot.mem[2][0] = s4*s6*(c2*s3 + c3*s2) - c6*(s5*(c2*c3 - s2*s3) + c4*c5*(c2*s3 + c3*s2));
	
	temp.rot.mem[0][1] = c6*(c4*s1 + s4*(c1*s2*s3 - c1*c2*c3)) - s6*(c5*(s1*s4 - c4*(c1*s2*s3 - c1*c2*c3)) - s5*(c1*c2*s3 + c1*c3*s2));
	temp.rot.mem[1][1] = s6*(c5*(c1*s4 + c4*(s1*s2*s3 - c2*c3*s1)) + s5*(c2*s1*s3 + c3*s1*s2)) - c6*(c1*c4 - s4*(s1*s2*s3 - c2*c3*s1));
	temp.rot.mem[2][1] = s6*(s5*(c2*c3 - s2*s3) + c4*c5*(c2*s3 + c3*s2)) + c6*s4*(c2*s3 + c3*s2);
	
	temp.rot.mem[0][2] = - s5*(s1*s4 - c4*(c1*s2*s3 - c1*c2*c3)) - c5*(c1*c2*s3 + c1*c3*s2);
	temp.rot.mem[1][2] = s5*(c1*s4 + c4*(s1*s2*s3 - c2*c3*s1)) - c5*(c2*s1*s3 + c3*s1*s2);
	temp.rot.mem[2][2] = c4*s5*(c2*s3 + c3*s2) - c5*(c2*c3 - s2*s3);
	
	
	temp.pos.dx = KR5ARC_A1*c1 + KR5ARC_A2*c1*c2 - KR5ARC_A3*c1*s2*s3 + KR5ARC_A3*c1*c2*c3 - KR5ARC_D4*c1*c2*s3 - KR5ARC_D4*c1*c3*s2;
	temp.pos.dy = KR5ARC_A1*s1 + KR5ARC_A2*c2*s1 - KR5ARC_D4*c2*s1*s3 - KR5ARC_D4*c3*s1*s2 - KR5ARC_A3*s1*s2*s3 + KR5ARC_A3*c2*c3*s1;		
	temp.pos.dz = - KR5ARC_D4*c2*c3 - KR5ARC_A2*s2 - KR5ARC_A3*c2*s3 - KR5ARC_A3*c3*s2 + KR5ARC_D4*s2*s3 + KR5ARC_D1;
	
	t6 = temp;    
	return 0;
}

//************************************
// Method:    InverseRobot
// FullName:  InverseRobot
// Access:    public 
// Returns:   extern "C"
// Qualifier: UINT KR5ARC_InverseRobot(const JAngle& JA,TRANS& t6)
// Parameter: KR5ARC机器人逆解
// Time:      2014/12/23
// Author:    杨明亮
// 东南大学智能机器人实验室
//************************************
bool KR5ARC_InverseRobot(JAngle& Jointangle,const JAngle& lastJointangle,const TRANS& t6)
{
	JAngle joint_temp;
	TRANS trans_temp = t6;  
	
	double px=trans_temp.pos.dx;
	double py=trans_temp.pos.dy;
	double pz=trans_temp.pos.dz-KR5ARC_D1;  
	double nx=trans_temp.rot.mem[0][0];  
	double ny=trans_temp.rot.mem[1][0];
	double nz=trans_temp.rot.mem[2][0];
	double ox=trans_temp.rot.mem[0][1];
	double oy=trans_temp.rot.mem[1][1];
	double oz=trans_temp.rot.mem[2][1];
	double ax=trans_temp.rot.mem[0][2];
	double ay=trans_temp.rot.mem[1][2];
	double az=trans_temp.rot.mem[2][2];

	// 求Angle1
	double temp11,temp12,temp1;
	temp11=atan2(py,px);		
	if(temp11>0) 
	{
		temp12=temp11-PI;
	}
	else
	{
		temp12=temp11+PI;
	}
	if((fabs(temp11-PI*lastJointangle.angle[0]/180))>(fabs(temp12-PI*lastJointangle.angle[0]/180)))
	{
		temp1=temp12;
	}
	else
	{
		temp1=temp11;
	}

	//求Angle3
	double temp31,temp32,temp3;
	double c1=cos(temp1);
	double s1=sin(temp1);
			
	double h=px*px+py*py+pz*pz+KR5ARC_A1*KR5ARC_A1;
	double g=2*KR5ARC_A1*c1*px+2*KR5ARC_A1*s1*py+KR5ARC_A3*KR5ARC_A3+KR5ARC_D4*KR5ARC_D4+KR5ARC_A2*KR5ARC_A2;
	double k=(h-g)/2/KR5ARC_A2;

	if (KR5ARC_A3*KR5ARC_A3+KR5ARC_D4*KR5ARC_D4-k*k < 0.0) {
		Jointangle = lastJointangle;
		return -1;
	}
 	temp31=atan2(KR5ARC_A3,KR5ARC_D4)-atan2(k,sqrt(KR5ARC_A3*KR5ARC_A3+KR5ARC_D4*KR5ARC_D4-k*k));
	if (temp31>PI)
	{
		temp31=temp31-2*PI;
	}
	else if (temp31<-PI)
	{
		temp31=temp31+2*PI;
	}

	temp32=atan2(KR5ARC_A3,KR5ARC_D4)-atan2(k,-sqrt(KR5ARC_A3*KR5ARC_A3+KR5ARC_D4*KR5ARC_D4-k*k));		
	if (temp32>PI)
	{
		temp32=temp32-2*PI;
	}
	else if (temp32<-PI)
	{
		temp32=temp32+2*PI;
	}
	
	if((fabs(temp31-PI*lastJointangle.angle[2]/180))>(fabs(temp32-PI*lastJointangle.angle[2]/180)))
	{	
		temp3=temp32;
	}
	else
	{
		temp3=temp31;
	}

	//求Angle2
	double c3=cos(temp3);
	double s3=sin(temp3);
	double temp2;   
	double s23=((-KR5ARC_A3-KR5ARC_A2*c3)*pz+(c1*px+s1*py-KR5ARC_A1)*(KR5ARC_A2*s3-KR5ARC_D4))/(pz*pz+(c1*px+s1*py-KR5ARC_A1)*(c1*px+s1*py-KR5ARC_A1));
	double c23=((-KR5ARC_D4+KR5ARC_A2*s3)*pz+(c1*px+s1*py-KR5ARC_A1)*(KR5ARC_A2*c3+KR5ARC_A3))/(pz*pz+(c1*px+s1*py-KR5ARC_A1)*(c1*px+s1*py-KR5ARC_A1));	
	temp2=atan2(s23,c23)-temp3;
	if (temp2>PI)
	{
		temp2=temp2-PI*2;
	}
	else if (temp2<-PI)
	{
		temp2=temp2+PI*2;
	}
	//求Angle4
	double temp41,temp42,temp4,temp5,temp6;
	double c2=cos(temp2);
	double s2=sin(temp2);
    temp41=atan2(-ax*s1+ay*c1,-ax*c1*c23-ay*s1*c23+az*s23);		
	if(temp41>0) 
	{
		temp42=temp41-PI;
	}
	else
	{
		temp42=temp41+PI;
	}
	if((fabs(temp41-PI*lastJointangle.angle[3]/180))>(fabs(temp42-PI*lastJointangle.angle[3]/180)))
	{	
		temp4=temp42;
	}
	else
	{
		temp4=temp41;
	}
	//求Angle5
	double c4=cos(temp4);
	double s4=sin(temp4);
	double c5=-(ax*c1*s23+ay*s1*s23+az*c23);
	double s5=-ax*(c1*c23*c4+s1*s4)-ay*(s1*c23*c4-c1*s4)+az*s23*c4;
	temp5=atan2(s5,c5);		

	//求Angle6
	double s6=-nx*(c1*c23*s4-s1*c4)-ny*(s1*c23*s4+c1*c4)+nz*s23*s4;
	double c6=-ox*(c1*c23*s4-s1*c4)-oy*(s1*c23*s4+c1*c4)+oz*s23*s4;				
	temp6=atan2(s6,c6);

	//赋值
	joint_temp.angle[0]=temp1*180/PI;
	joint_temp.angle[1]=temp2*180/PI;
	joint_temp.angle[2]=temp3*180/PI;
	joint_temp.angle[3]=temp4*180/PI;
    joint_temp.angle[4]=temp5*180/PI;
	joint_temp.angle[5]=temp6*180/PI;
	
	Jointangle = joint_temp;

	return 0;
}

KR5ARC_RKA::KR5ARC_RKA() : dh1(0, 2151, 0), dh2(0, -505, 90)
{
	DHparameters DH1(0.0, 675.0, 0.0);
	DHparameters DH2(260.0, 0.0, -90.0);
	DHparameters DH3(680.0, 0.0, 0.0);
	DHparameters DH4(35.0, -670.0, -90.0);
	DHparameters DH5(0.0, 0.0, 90.0);
	DHparameters DH6(0.0, 0.0, -90.0);
	DH.push_back(DH1);
	DH.push_back(DH2);
	DH.push_back(DH3);
	DH.push_back(DH4);
	DH.push_back(DH5);
	DH.push_back(DH6);
}

bool KR5ARC_RKA::PositiveRobot(const JAngle& JA, TRANS& t6)
{
	return KR5ARC_PositiveRobot(JA, t6);
}

bool KR5ARC_RKA::InverseRobot(JAngle& JA, const JAngle& lastJA, const TRANS& t6)
{
	return KR5ARC_InverseRobot(JA, lastJA, t6);
}

bool KR5ARC_RKA::InverseRobotEx(vector<JAngle>& vecJointangle, const TRANS& t6)
{
	return false;
}

illegal_joint KR5ARC_RKA::IsLegal(const JAngle& JA)
{
	if (JA.angle[0]<KR5ARCMIN_1 || JA.angle[0]>KR5ARCMAX_1 )
	{
		return JOINT1;
	}
	if (JA.angle[1]<KR5ARCMIN_2 || JA.angle[1]>KR5ARCMAX_2 )
	{
		return JOINT2;
	}
	if (JA.angle[2]<KR5ARCMIN_3 || JA.angle[2]>KR5ARCMAX_3)
	{
		return JOINT3;
	}
	if (JA.angle[3]<KR5ARCMIN_4|| JA.angle[3]>KR5ARCMAX_4 )
	{
		return JOINT4;
	}
	if (JA.angle[4]<KR5ARCMIN_5 || JA.angle[4]>KR5ARCMAX_5)
	{
		return JOINT5;
	}
	if (JA.angle[5]<KR5ARCMIN_6 || JA.angle[5]>KR5ARCMAX_6 )
	{
		return JOINT6;
	}
	return NOJOINT;
}

TRANS KR5ARC_RKA::get_trans_to_base(int index, const JAngle& angle)
{
	TRANS tmp;
	for (int i = 0; i <= index; i++) {
		tmp = tmp * DH[i].get_reftrans(angle.get_angle(i + 1));
	}

	return tmp;
}

TRANS KR5ARC_RKA::RotateX(double angle)
{
	TRANS res;
	res.rotx(angle);
	return res;
}

TRANS KR5ARC_RKA::RotateY(double angle)
{
	TRANS res;
	res.roty(angle);
	return res;
}

TRANS KR5ARC_RKA::RotateZ(double angle)
{
	TRANS res;
	res.rotz(angle);
	return res;
}

TRANS KR5ARC_RKA::Trans(double x, double y, double z)
{
	TRANS res;
	res.trans(x, y, z);
	return res;
}

TRANS KR5ARC_RKA::get_trans_to_world(int index, const JAngle& angle, const JAngle& ex_angle)
{
	assert(index >= 0 && index < 10);
	TRANS C0=Trans(0, -675.0, 0)*RotateZ(-90)*RotateX(90); 
	TRANS C1=Trans(0, 176.0, 0)*RotateY(90)*RotateX(-90)*RotateZ(ex_angle.angle[2]);
	TRANS C2=Trans(0, -388.0, 2822.0)*RotateX(90)*Trans(0, ex_angle.angle[3], 0);
	if (index == 0) {
		return C0 * C1;
	} else if (index == 1) {
		return C0*C1*C2;
	} else if (index == 2) {
		return C0*C1*C2*Trans(0.0, 78.0, 1323.0)*RotateY(-90)*RotateX(90);
	} else if (index < 9) {
		return C0*C1*C2*Trans(0.0, 78.0, 1323.0)*RotateY(-90)*RotateX(90)*get_trans_to_base(index - 3, angle);
	} else {
		TRANS T6ToFlange(0.0, 0.0, -1.0, 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -158.0);
		TRANS flangleToEnd = Trans(457.64, 0.0, 54.85)*RotateY(112.0);
		return C0*C1*C2*Trans(0.0, 78.0, 1323.0)*RotateY(-90)*RotateX(90)*get_trans_to_base(5, angle) * T6ToFlange;
	}
}

// TRANS KR5ARC_RKA::getTransWorldToWorkpiece(int index, const JAngle& ex_angle)
// {
// 	assert(index < 4);
// 	double ext1 = ex_angle.angle[0];
// 	double ext2 = ex_angle.angle[1];
// 	TRANS t[4];
// 	t[0] = Trans(2625.0, 1020.0, 1720.0)*RotateY(90)*RotateX(90);
// 	t[1] = Trans(0, 0, 106)*RotateZ(-90)*RotateZ(ext1);
// 	t[2] = Trans(0.0,505.0,2045.0)*RotateX(90)*RotateZ(ext2);

// 	TRANS ret;
// 	for (int i = 0; i <= index; i++) {
// 		ret = ret * t[i];
// 	}
// 	//return TP2*RotateZ(90);
// 	return ret;
// }


TRANS KR5ARC_RKA::getTransWorldToWorkpiece(int id, int index, const JAngle& ex_angle)
{
	assert(index < 4);
	double ext1;
	double ext2;

	TRANS t[4];
	ext1 = ex_angle.angle[0];
	ext2 = ex_angle.angle[1];
	if (id == 1) {
		t[0] = Trans(2625.0, 1020.0, 1720.0)*RotateZ(180)*RotateX(-90);
	} else if (id == 0) {
		t[0] = Trans(-2625.0, 1020.0, 1720.0)*RotateZ(180)*RotateX(-90);
		// ext1 = ex_angle.angle[4];
		// ext2 = ex_angle.angle[5];
	}
	
//	t[1] = Trans(0, 0, 106)*RotateZ(-90)*RotateZ(ext1);
//	t[2] = Trans(0.0,505.0,2045.0)*RotateX(90)*RotateZ(ext2);
	t[1] = dh1.get_reftrans(ext1);
	t[2] = dh2.get_reftrans(ext2);

	TRANS ret;
	for (int i = 0; i <= index; i++) {
		ret = ret * t[i];
	}
	//return TP2*RotateZ(90);
	return ret;
}

/*
	2014年12月23日 杨明亮
*/
