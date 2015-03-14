#ifndef _WELD_POINT_H_
#define _WELD_POINT_H_
#include "robotdata.h"

class weld_point
{
public:
	Vector3D m_p;
	Vector3D m_n;
	Vector3D m_t;
	double	 m_theta; //보룹허실
	double	 m_fi;    //보룹瘻실
	JAngle m_angle1;
	JAngle m_angle2;
public:
	weld_point(Vector3D p, Vector3D n, Vector3D t, double theta = 0, double fi = 0)
		:m_p(p), m_n(n), m_t(t), m_theta(theta), m_fi(fi) {}
	~weld_point(){}

	TRANS get_trans();
	bool operator < (weld_point r_wedpoint);
};

class weld_point_compare
{
private:
	const bool m_minimize;
	
public:
	weld_point_compare( bool minimize )
		: m_minimize( minimize )
	{
	}
	
	bool operator()( weld_point& p1, weld_point& p2 )
	{	
		if (m_minimize)
		{
			return p1.m_angle1.angle[1] < p2.m_angle1.angle[1];
		} 
		else
		{
			return p1.m_angle1.angle[1] > p2.m_angle1.angle[1];
		}
		
	}
};


#endif