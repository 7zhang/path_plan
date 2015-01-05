#ifndef __KR5ARC_H_
#define __KR5ARC_H_

#include "robotkinematic.h"

#include <iostream>
#include <vector>
using namespace std;

///////////////////////////////////
//KR5ARC机器人杆件参数
///////////////////////////////////
#define KR5ARC_A1 260.0f
#define KR5ARC_A2 680.0f
#define KR5ARC_A3 35.0f
#define KR5ARC_D1 675.0f
#define KR5ARC_D4 -670.0f

//////////////////////////////////////
//KR5ARC机器人各关节的最大最小值
/////////////////////////////////////
#define KR5ARCMIN_1 -155.0f
#define KR5ARCMAX_1 155.0f
#define KR5ARCMIN_2 -180.0f
#define KR5ARCMAX_2 65.0f
#define KR5ARCMIN_3 -15.0f
#define KR5ARCMAX_3 158.0f
#define KR5ARCMIN_4 -350.0f
#define KR5ARCMAX_4 350.0f
#define KR5ARCMIN_5 -130.0f
#define KR5ARCMAX_5 130.0f
#define KR5ARCMIN_6 -350.0f
#define KR5ARCMAX_6 350.0f
/////////////////////////////////////

class KR5ARC_RKA : public IKinematicAlg             //KR5ARC机器人算法类
{
	
public:
	KR5ARC_RKA()  {}
	virtual ~KR5ARC_RKA()  {}
	virtual bool InverseRobot(JAngle& Jointangle,const JAngle& lastJointangle,const TRANS& t6);
	virtual bool InverseRobotEx(vector<JAngle>& vecJointangle, const TRANS& t6);
	virtual bool PositiveRobot(const JAngle& JA,TRANS& t6);
	virtual illegal_joint IsLegal(const JAngle& JA);	
};

#endif /*__KR5ARC_H_*/

/*
	2014年12月23日 杨明亮
*/
