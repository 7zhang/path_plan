// ***************************************************************
// *<Pre>
// *文件名：robotdata.h
// *功能描述：
// *作者：王成
// *日期：2011-3-7
// *</Pre>
// ***************************************************************
#ifndef _ROBOTDATA_H_
#define _ROBOTDATA_H_

#include "geometric.h"
#include <vector>

#define IN
#define OUT
#define INOUT

using namespace std;

class TRANS;
class RPY;
class Euler;	// ZYZ形式
class Quaternion;

// !@class
// ******************************************************************
// *<pre>
// *类名：JAngle
// *作者：王成
// *日期：2011-3-7
// *</pre>
// ******************************************************************

class  JAngle
{
public:
	double angle[6];     //单位为度
public:
	JAngle(double a1=0.0, double a2=0.0, double a3=0.0, double a4=0.0, double a5=0.0, double a6=0.0);
	JAngle(double a[]);

public:
	// [[ <FUNCTION GOUP>
	void set_angle(double value, int num);		// num: 1-6
	void set_angles(double ang1, double ang2, double ang3, double ang4, double ang5, double ang6);
	double get_angle(int num) const;
	// ]] </FUNCTION GOUP>

	bool operator!=(const JAngle& rhs)const;
	bool operator==(const JAngle& rhs)const;
};


// !@class
// ******************************************************************
// *<pre>
// *类名：TRANS
// *作者：王成
// *日期：2011-3-7
// *</pre>
// ******************************************************************
class  TRANS
{
public:
	Matrix3D rot;				// 方向描述：旋转矩阵
	Vector3D pos;				// 位置描述
public:
TRANS(double _nx=1.0, double _ny=0.0, double _nz=0.0, 	
      double _ox=0.0, double _oy=1.0, double _oz=0.0,
      double _ax=0.0, double _ay=0.0, double _az=1.0,		
      double _px=0.0, double _py=0.0, double _pz=0.0)
	:rot(_nx, _ox, _ax, _ny, _oy, _ay, _nz, _oz, _az), pos(_px, _py, _pz)
	{}
TRANS(const Matrix3D& _rot, const Vector3D& _p) 
	:rot(_rot), pos(_p)
	{}

public:
	void trans(double px, double py, double pz);	// 标架平移

	//------ ------------ -------
	void rotx(double angle);						// 标架旋转	
	void roty(double angle);
	void rotz(double angle);
	// --------------------------

	void inverse();									// 标架求逆

	bool Trans2RPY(RPY& rpy,const RPY& lastRPY) const;    //转换为RPY坐标
	bool Trans2RPY(RPY& rpy) const;
	bool Trans2Euler(Euler& Eu,const Euler& lastEu) const;//转换为欧拉坐标
	bool Trans2Euler(Euler& Eu) const;//转换为欧拉坐标
	bool Trans2Quat(Quaternion& quat) const;		// 转换为四元数

	// 计算RPY和Euler，方法实现同上，仅接口不同
	RPY GetRPY() const;		
	Euler GetEuler() const;
	
	Vector3D operator *(const Vector3D& _pos) const;		// 坐标变换
	Point3D operator *(const Point3D& _pos) const;		// 坐标变换
	TRANS operator * (const TRANS& t6) const;				// 标架乘法
	void operator *= (const TRANS& t6); 

};


// !@class
// ******************************************************************
// *<pre>
// *类名：RPY
// *先绕Z轴旋转角度orient.dz，再绕新的Y轴旋转角度orient.dy，最后绕新的X轴旋转角度orient.dx
// *作者：王成
// *日期：2011-3-7
// *</pre>
// ******************************************************************
class  RPY
{
public:
	Vector3D pos;
	Vector3D orient;   //单位为度
public:
RPY(double px=0.0, double py=0.0, double pz=0.0, double rx=0.0, double ry=0.0, double rz=0.0)
	:pos(px,py,pz),orient(rx,ry,rz)
	{}
RPY(const Vector3D& _pos, const Vector3D& _orient)
	:pos(_pos),orient(_orient)
	{}

public:
	void RPY2Trans(TRANS& t6);			// 转换为4*4矩阵
	void RPY2Quat(Quaternion& quat);	// 转换为四元数
};



// !@class
// ******************************************************************
// *<pre>
// *类名：Euler  ZYZ欧拉角
// *作者：王成
// *日期：2011-3-7
// *</pre>
// ******************************************************************
class  Euler
{
public:
	Vector3D pos;
	Vector3D orient;   //单位为度
public:
Euler(double px=0.0, double py=0.0, double pz=0.0, double rx=0.0, double ry=0.0, double rz=0.0)
	:pos(px,py,pz),orient(rx,ry,rz)
	{}
Euler(const Vector3D& _pos, const Vector3D& _orient)
	:pos(_pos),orient(_orient)
	{}

public:
	void Euler2Trans(TRANS& t6);
	void Euler2Quat(Quaternion& quat);	// 转换为四元数
};



// !@class
// ******************************************************************
// *@<pre>
// *@类名：RouteType,RoutePoint,RouteSeg
// *@作者：王成
// *@日期：2011-3-7
// *@</pre>
// ******************************************************************
enum RouteType{ JOINT_ROUTE, LINE_ROUTE, ARC_ROUTE,CIR_ROUTE, FLYBY_ROUTE, SPLINE_ROUTE };

struct  RoutePoint
{
	double vel;
	double acc;
	RPY rpy;
	JAngle J_Angle;
};

struct  RouteSeg
{
	RoutePoint beginPos;	//起始示教点
	RoutePoint midPos;		//中间示教点（用于圆弧插补中）
	RoutePoint endPos;		//终止示教点

	double length;
	double vel;
	double acc;
	double jerk;			//加加速度
	RouteType type;			//路径类型
};

// !@class
// ******************************************************************
// *<pre>
// *类名：Quaternion
// *功能：四元数类
// *作者：葛浏昊
// *日期：20112-4-26
// *</pre>
// ******************************************************************
class  Quaternion
{
public:
	double w,x,y,z;

public:
Quaternion(double _w=0.0, double _x=0.0, double _y=0.0, double _z=0.0)
	:w(_w),x(_x),y(_y),z(_z)
	{}
	Quaternion(RPY _rpy)
	{
		//Quaternion q;
		_rpy.RPY2Quat( *this );
	}
	Quaternion(TRANS _trans)
	{
		//Quaternion q;
		_trans.Trans2Quat( *this );
	}
	
public:
	bool Quat2RPY(RPY& rpy);
	void Quat2TRANS(TRANS& trans);

	Quaternion& operator+=(const Quaternion& rhs);
	double mod() const;									// 求模
	Quaternion Conjugate() const;					// 求共轭
	Quaternion Inverse() const;						// 求逆

	static Quaternion Double(Quaternion p, Quaternion q);
	static Quaternion Bisect(Quaternion p, Quaternion q);
};

Quaternion operator+(const Quaternion& lhs,const Quaternion& rhs);
Quaternion operator-(const Quaternion& lhs,const Quaternion& rhs);
double operator*(const Quaternion& lhs,const Quaternion& rhs);			// 四元数的点乘
Quaternion operator^(const Quaternion& lhs,const Quaternion& rhs);			// 四元数的直乘
Quaternion operator*(const double& lhs,const Quaternion& rhs);
Quaternion operator*(const Quaternion& lhs,const double& rhs);
Quaternion operator/(const Quaternion& lhs,const double& rhs);

Vector3D quat_log(const Quaternion& quat);
Quaternion quat_exp(const Vector3D& vec);

// 四元数球面插补
bool Quat_Interp(Quaternion& Q_result, const Quaternion& Q1,const Quaternion& Q2,double r);
bool Quat_Bez3rdInterp(Quaternion& Q_result, const Quaternion& Q1,const Quaternion& Q2,const Quaternion& a1,const Quaternion& b2,double u);  
bool Quat_ArcInterp(Quaternion& Q_result, const Quaternion& Q1,const Quaternion& Q2,const Quaternion& Q3,double cur_arc,double arc1,double arc2);  
bool Quat_Bez2ndInterp( Quaternion& Q_result, const Quaternion& Q1,const Quaternion& Q2,const Quaternion& Q3,double u ) /* 二阶贝塞尔曲线 */;

// B样条曲线
double BSplineBasicFun(int i, int k, const vector<double>& u, double t);
double BSplineCumulBasicFun(int i, int k, const vector<double>& u, double t);
bool Quat_BSplineInterp( Quaternion& Q_result, const vector<Quaternion>& quat_list, double t, int num );


#endif /* _ROBOTDATA_H_ */
