#ifndef _POSITIONER_H_
#define _POSITIONER_H_

#include "dh_parameter.h"
#include "weld_point.h"
#include <vector>

class positioner
{
public:
	std::vector<DHparameters> DH;
	//vector<JAngle> m_jangle1;
	//vector<JAngle> m_jangle2;
public:
	positioner();
	~positioner(){}

	int InverseRobot(weld_point& weldpoint, vector<JAngle>& vecJointangle);

	//bool InverseRobotEx(vector<JAngle>& vecJointangle, const TRANS& t6);

	bool PositiveRobot(weld_point& weldpoint, const JAngle& JA);
};



#endif
