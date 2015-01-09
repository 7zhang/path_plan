///////////////////////////////////////////////////////////
//  IKinematicAlg.h
//  Implementation of the Interface IKinematicAlg
//  Created on:      08-三月-2011 11:08:11
//  Original author: Administrator
///////////////////////////////////////////////////////////

#ifndef _ROBOTKINEMATIC_H_
#define _ROBOTKINEMATIC_H_

#include "robotdata.h"
#include <vector>
using namespace std;

//一些固定数值，应该放在文件中读入内存中，这里仅仅是计算轴的长度

// #define EXPENSIN 0.00001
// const double Ts=0.01;  //步长

//KunShan机器人杆件参数
///////////////////////////////////
/*#define KUNSHANROBOT_D1 570.0f
#define KUNSHANROBOT_A2 280.0f
#define KUNSHANROBOT_A3 620.0f
#define KUNSHANROBOT_D4 540.0f
#define KUNSHANROBOT_D5 100.0f */
#define KUNSHANROBOT_D1 729.41f
#define KUNSHANROBOT_A2 306.03f
#define KUNSHANROBOT_A3 614.0f
#define KUNSHANROBOT_D4 524.0f
#define KUNSHANROBOT_D5 82.0f 
/////////////////////////////////////
enum illegal_joint { NOJOINT, JOINT1, JOINT2, JOINT3, JOINT4, JOINT5, JOINT6};

class  IKinematicAlg
{
	
public:
	IKinematicAlg() {
		
	}
	
	virtual ~IKinematicAlg() {
		
	}
	
	virtual bool InverseRobot(JAngle& Jointangle,const JAngle& lastJointangle,const TRANS& t6) =0;
	virtual bool InverseRobotEx(vector<JAngle>& vecJointangle, const TRANS& t6)=0;
	virtual bool PositiveRobot(const JAngle& JA,TRANS& t6) =0;
	virtual illegal_joint IsLegal(const JAngle& JA)=0;
};

class  KunshanRKA : public IKinematicAlg    //昆山一号机器人算法类
{
	
public:
	KunshanRKA()  {}
	virtual ~KunshanRKA()  {}
	
	virtual bool InverseRobot(JAngle& Jointangle,const JAngle& lastJointangle,const TRANS& t6);
	virtual bool InverseRobotEx(vector<JAngle>& vecJointangle, const TRANS& t6);
	virtual bool PositiveRobot(const JAngle& JA,TRANS& t6);
	virtual illegal_joint IsLegal(const JAngle& JA);
};




// !@class
// ******************************************************************
// *@<pre>
// *@类名：JResolution  机器人逆解类
// *@作者：葛浏昊
// *@日期：2011-3-30
// *@</pre>
// ******************************************************************
class JResolution          
{
public:
	JResolution();
	virtual ~JResolution();
public:
	JAngle m_Resolution[8];
	char err[8];            //判断关节角是否正确的变量，正确为0，不正确为1，奇异解为2
};
	
	//[[----------根据参考关节角refAng选取合适的JAngle----------------
	 JAngle SelectAngle(vector<JAngle>& vecJointangle, JAngle refAng);
	//----------------------------------------------------------------]]

// [[ <FUNCTION GROUP>
	//昆山一号机器人正解
	 bool KS_PositiveRobot(const JAngle& JA,TRANS& t6);  
	//昆山一号机器人逆解
	 bool KS_InverseRobot(JAngle& Jointangle,const JAngle& lastJointangle,const TRANS& t6);
	//昆山一号机器人逆解（返回8组解）		
	 bool KSNew_InverseRobot(JResolution& resolution, const TRANS& t6);							
// ]] </FUNCTION GROUP>

#endif /* _ROBOTKINEMATIC_H_ */
