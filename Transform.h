// Transform.h: interface for the Transform class.
// Author: 王海波
// 功能：用来得到各个子模型坐标系与世界坐标系的变换关系；第二层就是把离线编程平台的相关函数进行完善
//////////////////////////////////////////////////////////////////////
#ifndef _TRANSFORM_H_
#define _TRANSFORM_H_


#include "robotdata.h"
#include "robotkinematic.h"
//#include "robotmotion.h"
#include <stdlib.h>

extern double BETALIMIT;
inline const JAngle operator+(const JAngle& lhs, const JAngle& rhs)
{
	JAngle res;
	for (int i = 0; i < 6; ++i)
	{
		res.angle[i] = lhs.angle[i] + rhs.angle[i];
	}
	return res;
}
inline const JAngle operator-(const JAngle& lhs, const JAngle& rhs)
{
	JAngle res;
	for (int i = 0; i < 6; ++i)
	{
		res.angle[i] = lhs.angle[i] - rhs.angle[i];
	}
	return res;
}
inline const JAngle operator/(const JAngle& lhs, double ratio)
{
	JAngle res;

	for (int i = 0; i < 6; ++i)
	{
		res.angle[i] = lhs.angle[i] / ratio;
	}
	return res;
}
inline const JAngle operator*(const JAngle& lhs, double ratio)
{
	JAngle res;
	for (int i = 0; i < 6; ++i)
	{
		res.angle[i] = lhs.angle[i] * ratio;
	}
	return res;
}
//不做差再比较两个JAngle是没有意义的,这里对大关节进行加权，比较符合大关节不宜运动太多的实际情况
inline bool operator<(const JAngle& lhs, const JAngle& rhs)
{
	double totalAngle1 = 0.0;
	double totalAngle2 = 0.0;
	double weights[6] = {1.8, 1.7, 1.7, 1.4, 1.0, 1.0};//这里可以进行整定
	for (int i = 0; i < 6; ++i)
	{
		totalAngle1 += fabs(lhs.angle[i]) * weights[i];
		totalAngle2 += fabs(rhs.angle[i]) * weights[i];
	}
	return totalAngle1 < totalAngle2;
}

//关节角范围有没有在昆山一号的有效范围内
inline bool isJAngleInside(const JAngle& jAngle)
{
	return ((-150.0 < jAngle.angle[0]) && (jAngle.angle[0] < 180.0))
		&& ((-125.0 < jAngle.angle[1]) && (jAngle.angle[1] < 30.0))
		&& ((-120.0 < jAngle.angle[2]) && (jAngle.angle[2] < 150.0))
		&& ((-180.0 < jAngle.angle[3]) && (jAngle.angle[3] < 180.0))
		&& ((-120.0 < jAngle.angle[4]) && (jAngle.angle[4] < 120.0))
		&& ((-180.0 < jAngle.angle[5]) && (jAngle.angle[5] < 180.0));
}
//比较两个关节角哪个离参考关节角较近的函数对象
class JAngleComparer
{
public:
JAngleComparer(const JAngle & reference): m_referenceJAngle(reference)
	{
		m_bIsThereRefrence = true;
	}
	JAngleComparer()//如果没有指定参考关节角，则用来判断两个关节角本身是否足够接近
	{
		m_bIsThereRefrence = false;
	}

	bool operator()(const JAngle& lhs, const JAngle& rhs)const
	{
		if (!m_bIsThereRefrence)
		{
//			throw std::runtime_error("没有比较意义！请确定参考量");
		}
		JAngle diff1 = lhs - m_referenceJAngle;
		JAngle diff2 = rhs - m_referenceJAngle;
		return diff1 < diff2;
	}
	bool isNearby(const JAngle& lhs, const JAngle& rhs, double Mneighbor = 80.0)const
	{//返回两个机器人位姿是否足够接近
		JAngle diff = lhs - rhs;
		double weights[6] = {1.3, 1.3, 1.3, 1.1, 1.0, 1.0};
		double total_diff = 0.0;
		for (int i = 0; i < 6; ++i)
		{
			total_diff += pow(diff.angle[i] * weights[i], 2);
		}
		total_diff = sqrt(total_diff);
		if (total_diff < Mneighbor)
		{
			return true;
		}
		return false;
	}

private:
	JAngle m_referenceJAngle;//参考关节角
	bool m_bIsThereRefrence;
};

//注意参数里涉及到角度都不是弧度，是角度，只是在用math库是需要转换到弧度
//长度单位是毫米
//莱斯机器人专用
class Transform  
{
public:
	Transform();
	virtual ~Transform();

public:
	static void getTransBaseToJoints(const JAngle& jAngle, 
					 TRANS& T01, TRANS& T02, TRANS& T03, TRANS& T04, TRANS& T05, TRANS& T06); //获得机器人基座标系到各个关节坐标系的齐次变换矩阵

	static void getTransWorldToPos(const JAngle& extJAngle, TRANS& TP1, TRANS& TP2);//获得世界坐标系到变位机的齐次变换矩阵

	static TRANS getTransWorldToRail(const JAngle& extJAngle);//获得世界坐标系到杨力导轨的齐次变换矩阵
	static TRANS getTransWorldToBase(const JAngle& extJAngle);//获得世界坐标系到机器人基座的齐次变换矩阵
	//static TRANS getTransWorldToPositioner();//获得世界坐标系到变位机的齐次变换矩阵
	static TRANS getTransWorldToWorkpiece(const JAngle& extJAngle);//获得世界坐标系到工件坐标系的齐次变换矩阵
	static TRANS getTrans6ToFlange();//第六关节到法兰盘的T
	static TRANS getTransFlangeToTorch();//法兰盘到焊枪末尾的T
	static TRANS getTrans6ToTorch();//第六关节到焊枪末端坐标系
	static TRANS getTrans6ToGun();//第六关节到焊枪坐标系

	static TRANS getTransWorldToTorch(const JAngle& extJAngle, const TRANS& TWorkpieceToSeam);//获得世界坐标系到焊枪末端坐标系的齐次变换矩阵
	static TRANS getTransBaseToTorch(const JAngle& extJAngle, const TRANS& TWorkpieceToSeam);//获得机器人基座坐标系到焊枪末端坐标系的齐次变换矩阵

	static TRANS getT6ByTransBaseToTorch(const TRANS& TBaseToTorch);//获得BaseTo6

	static TRANS rotateX(double angle);
	static TRANS rotateY(double angle);
	static TRANS rotateZ(double angle);
	static TRANS trans(double x, double y, double z);
};
//这里重新定义正逆解函数
TRANS positiveKSRobot(const JAngle& jAngle);                                       //昆山一号机器人正解
JAngle inverseKSRobot(const TRANS& t6, const JAngle& lastJointangle, bool& succeeded);//昆山一号机器人逆解：


#endif /* _TRANSFORM_H_ */
