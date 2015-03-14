#ifndef __DHPARAMETERS_H_
#define __DHPARAMETERS_H_
#include "robotdata.h"
class DHparameters
{
public:
	double a;
	double d;
	double alpha;
	//double m_theta;
public:
	DHparameters(double m_a , double m_d , double m_alpha):a(m_a),d(m_d),alpha(m_alpha){}
	TRANS get_reftrans( const double theta)
	{
		TRANS outframe;
		outframe.rot.mem[0][0] = cos(theta*PI/180);
		outframe.rot.mem[0][1] = -1 * sin(theta*PI/180);
		outframe.rot.mem[0][2] = 0;
		outframe.rot.mem[1][0] = sin(theta*PI/180) * cos(alpha*PI/180);
		outframe.rot.mem[1][1] = cos(theta*PI/180) * cos(alpha*PI/180);
		outframe.rot.mem[1][2] = -1 * sin(alpha*PI/180);
		outframe.rot.mem[2][0] = sin(theta*PI/180) * sin(alpha*PI/180);
		outframe.rot.mem[2][1] = cos(theta*PI/180) * sin(alpha*PI/180);
		outframe.rot.mem[2][2] = cos(alpha*PI/180);
		outframe.pos.dx = a;
		outframe.pos.dy = -1 * d * sin(alpha*PI/180);
		outframe.pos.dz = d * cos(alpha*PI/180);
		
		return outframe;
	}
};

#endif
