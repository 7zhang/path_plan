//// !@File
// ***************************************************************
// *<Pre>
// *文件名：geometric.h
// *功能描述：
// *作者：王成
// *日期：2011-3-4
// *</Pre>
// ***************************************************************


#ifndef __GEOMCALC_H__
#define __GEOMCALC_H__

#include <math.h>

//二维、三维点、向量、矩阵声明

class Point2D;
class Point3D;
class Vector2D;
class Vector3D;
class Matrix3D;
class Matrix4D;

//////////////////////////////////////////////////////////////////////////
//常量定义

const double ZERO = 1.0E-9;
const double PI = 3.1415926536;



inline bool isZero(double v)
{
	return (fabs(v) <= ZERO);
}

//////////////////////////////////////////////////////////////////////////


// !@class
// ******************************************************************
// *<pre>
// *类名：Point2D
// *作者：王成
// *日期：2011-3-4
// *</pre>
// ******************************************************************
class  Point2D
{
public:
	double x;
	double y;

	//----------------------构造函数-------------------------
public:
	Point2D(double ix=0.0,double iy=0.0);

public:
	// 点偏移一定量
	void offset(Vector2D vec2);
	void offset( double dx=0.0, double dy=0.0 );
} ;

// [[ <FUNCTION GOUP>
bool operator ==( Point2D& lpoint, Point2D& rpoint );
bool operator !=( Point2D& lpoint, Point2D& rpoint );

Point2D operator + ( const Point2D& lpoint, const Point2D& rpoint );
Point2D operator - ( const Point2D& lpoint, const Point2D& rpoint );
Point2D& operator += ( Point2D& lpoint, const Point2D& rpoint );
Point2D& operator -= ( Point2D& lpoint, const Point2D& rpoint );
// ]] </FUNCTION GOUP>



// !@class
// ******************************************************************
// *<pre>
// *类名：Point3D
// *作者：王成
// *日期：2011-3-4
// *modified by whb at 2011/12/22, 两个点的减改为向量，点加点改为点加向量，点减点改为点减向量
// *</pre>
// ******************************************************************
class  Point3D
{
public:
	double x;
	double y;
	double z;
public:
	Point3D(double ix=0.0, double iy=0.0, double iz=0.0);
	
public:

	// 点偏移一定量
	void offset(Vector3D vec3);
	void offset( double dx=0.0, double dy=0.0, double dz=0.0);
	//zoom,乘以一个比例
	void zoom(double rate);
};
// [[ <FUNCTION GOUP>
bool operator ==( Point3D& lpoint, Point3D& rpoint );
bool operator !=( Point3D& lpoint, Point3D& rpoint );
	
// 加减运算操作
// 	  Point3D operator + ( const Point3D& lpoint, const Point3D& rpoint );
// 	  Point3D operator - ( const Point3D& lpoint, const Point3D& rpoint );
// 	  Point3D& operator += ( Point3D& lpoint, const Point3D& rpoint );
// 	  Point3D& operator -= ( Point3D& lpoint, const Point3D& rpoint );

Point3D operator + ( const Point3D& lpoint, const Vector3D& v );
Vector3D operator - ( const Point3D& lpoint, const Point3D& rpoint );
Point3D operator - ( const Point3D& lpoint, const Vector3D& v );
Point3D& operator += ( Point3D& lpoint, const Vector3D& rpoint );
Point3D& operator -= ( Point3D& lpoint, const Vector3D& rpoint );

// ]] </FUNCTION GOUP>
// 比较操作



//////////////////////////////////////////////////////////////////////////////
// !@class
// ******************************************************************
// *<pre>
// *类名：Vector2D
// *作者：王成
// *日期：2011-3-4
// *</pre>
// ******************************************************************
class  Vector2D 
{
public:
	double dx;
	double dy;

public:
	Vector2D(double ix=0.0,double iy=0.0);

public:
	void unitize();      //归一化处理
	double get_length(); //求向量长度
};

// [[ <FUNCTION GOUP>
Vector2D operator + ( const Vector2D& lvec, const Vector2D& rvec );
Vector2D operator - ( const Vector2D& lvec, const Vector2D& rvec );
Vector2D& operator += ( Vector2D& lvec, const Vector2D& rvec );
Vector2D& operator -= ( Vector2D& lvec, const Vector2D& rvec );

Vector2D operator * ( double factor, const Vector2D& vec );
Vector2D operator * ( const Vector2D& vec, double factor );
Vector2D& operator *= (Vector2D& vec, double factor);

Vector2D operator / ( const Vector2D& vec, double factor );
Vector2D& operator /= (Vector2D& vec, double factor);

double operator ^( const Vector2D& lvec, const Vector2D& rvec );			// 点乘
// ]] </FUNCTION GOUP>




// !@class
// ******************************************************************
// *<pre>
// *类名：Vector3D
// *作者：王成
// *日期：2011-3-4
// *</pre>
// ******************************************************************
class  Vector3D
{
public:
	double dx;
	double dy;
	double dz;

public:
	Vector3D( double dx=0.0, double dy=0.0, double dz=0.0 );

public:
	// void offset( double dx, double dy, double dz );
	void unitize();
	double get_length();
// 	double GetLengthXY() const;
};

// [[ <FUNCTION GOUP>
Vector3D operator + ( const Vector3D& lvec, const Vector3D& rvec );
Vector3D operator - ( const Vector3D& lvec, const Vector3D& rvec );
Vector3D& operator += ( Vector3D& lvec, const Vector3D& rvec );
Vector3D& operator -= ( Vector3D& lvec, const Vector3D& rvec );

Vector3D operator * ( double factor, const Vector3D& vec );
Vector3D operator * ( const Vector3D& vec, double factor );
Vector3D& operator *= (Vector3D& vec, double factor);

Vector3D operator / ( const Vector3D& vec, double factor );
Vector3D& operator /= (Vector3D& vec, double factor);

Vector3D operator * ( const Vector3D& lvec, const Vector3D& rvec );		// 向量叉乘
double operator ^( const Vector3D& lvec, const Vector3D& rvec );			// 点乘
// ]] </FUNCTION GOUP>

// !@class
// ******************************************************************
// *<pre>
// *类名：Vector3D
// *作者：王成
// *日期：2011-3-4
// *</pre>
// ******************************************************************
class  Vector4D
{
public:
	double mem[4];

public:
	Vector4D( double m1=0.0, double m2=0.0, double m3=0.0, double m4=0.0 );

public:
	// void offset( double dx, double dy, double dz );
//	void unitize();
//	double get_length();
// 	double GetLengthXY() const;
};

// [[ <FUNCTION GOUP>
Vector4D operator + ( const Vector4D& lvec, const Vector4D& rvec );
Vector4D operator - ( const Vector4D& lvec, const Vector4D& rvec );
Vector4D& operator += ( Vector4D& lvec, const Vector4D& rvec );
Vector4D& operator -= ( Vector4D& lvec, const Vector4D& rvec );

Vector4D operator * ( double factor, const Vector4D& vec );
Vector4D operator * ( const Vector4D& vec, double factor );
Vector4D& operator *= (Vector4D& vec, double factor);

Vector4D operator / ( const Vector4D& vec, double factor );
Vector4D& operator /= (Vector4D& vec, double factor);
// ]] </FUNCTION GOUP>

// !@class
// ******************************************************************
// *<pre>
// *类名：Matrix3D
// *作者：王成
// *日期：2011-3-5
// *</pre>
// ******************************************************************
class  Matrix3D
{
public:
	double mem[3][3];
public:
	Matrix3D();
	Matrix3D(double (&m)[3][3]);
	Matrix3D(double (&m)[9]);
	Matrix3D(double d1,double d2,double d3,double d4,double d5,double d6,double d7,double d8,double d9)
	{
		mem[0][0] = d1;	mem[0][1] = d2;	mem[0][2] = d3;	
		mem[1][0] = d4; mem[1][1] = d5;	mem[1][2] = d6;
		mem[2][0] = d7;mem[2][1] = d8; mem[2][2] = d9;
	}
	Matrix3D(const Matrix3D& m);
public:
	void transpos();		// 矩阵转置
	void inverse();			// 矩阵求逆
	double get_determinant();	// 行列式值
	void jacobbi(Matrix3D& v, double* pArray)const;//通过Jocobbi迭代计算特征根与特征向量，矩阵v为特征向量矩阵， pArray为为特征值
};
// [[ <FUNCTION GOUP>
Matrix3D operator+( const Matrix3D& lma, const Matrix3D& rma );
Matrix3D operator-( const Matrix3D& lma, const Matrix3D& rma );
Matrix3D& operator+=( Matrix3D& lma, const Matrix3D& rma );
Matrix3D& operator-=( Matrix3D& lma, const Matrix3D& rma );
Matrix3D operator*( const Matrix3D& lma, const Matrix3D& rma );
Matrix3D& operator*=( Matrix3D& lma, const Matrix3D& rma );
Vector3D operator*( const Matrix3D& lma, const Vector3D& rvec);

Matrix3D operator*(double factor, const Matrix3D& ma);
Matrix3D operator*(const Matrix3D& ma, double factor);
Matrix3D& operator*=(Matrix3D& ma, double factor);

Matrix3D operator/(const Matrix3D& ma, double factor);
Matrix3D& operator/=(Matrix3D& ma, double factor);
// ]] </FUNCTION GOUP>


// 	static Matrix3D CreateMirrorMatrix(VECTOR3D plnNorm);
// 	static Matrix3D CreateRotateMatrix(double da,VECTOR3D bv);
// 	static Matrix3D CreateScaleMatrix(double);
// 	static Matrix3D CreateTransfMatrix(VECTOR3D vec);

// !@class
// ******************************************************************
// *<pre>
// *类名：Matrix4D
// *描述：齐次变换矩阵
// *作者：葛浏昊
// *日期：2011-3-17
// *</pre>
// ******************************************************************
class  Matrix4D
{
public:
	double mem[4][4];
//	Matrix3D rot;
//  Vector3D pos;
public:
	Matrix4D();
	Matrix4D(double (&m)[4][4]);
	Matrix4D(const Matrix3D& rot,const Vector3D& pos);
	Matrix4D(const Matrix4D& m);
public:
	void transpos();		// 矩阵转置
	void inverse();			// 矩阵求逆
	double get_determinant();	// 行列式值
	void jacobbi(Matrix4D& v, double* pArray)const;//通过Jocobbi迭代计算特征根与特征向量，矩阵v为特征向量矩阵， pArray为为特征值
public:
//	Matrix3D get_rot(double mem[4][4]);
//	Vector3D get_pos(double mem[4][4]);
};
// [[ <FUNCTION GOUP>
Matrix4D operator*( const Matrix4D& lma, const Matrix4D& rma );
Matrix4D& operator*=( Matrix4D& lma, const Matrix4D& rma );
Vector4D operator*( const Matrix4D& lma, const Vector4D& rvec);
Vector4D& operator*=( Vector4D& point,const Matrix4D& matrix );
Point3D operator*(const Matrix4D& matrix, const Point3D& point);
Point3D& operator*=(Point3D& point , const Matrix4D& matrix);
Vector3D operator*(const Matrix4D& matrix, const Vector3D& vec);
Vector3D& operator*=(Vector3D& vec , const Matrix4D& matrix);
// ]] </FUNCTION GOUP>


// exported API functions	AFX_EXT_API
double	 _AngleBetween(Vector2D v1,Vector2D v2);
double	 _AngleBetween(Vector3D v1,Vector3D v2);
double	 _DistOf(Point2D pt0, Point2D pt1);
double	 _DistOf(Point3D pt0, Point3D pt1);
bool	 _IsParallel(Vector2D v0,Vector2D v1);
bool	 _IsParallel(Vector3D v0,Vector3D v1);
bool	 _IsOrthogonal(Vector2D v0,Vector2D v1);
bool	 _IsOrthogonal(Vector3D v0,Vector3D v1);
// 由平移和旋转量获得变换矩阵
//void     _GetTransferMatrix(double x,double y,double z,double Rx,double Ry,double Rz,Matrix4D& T);

#endif
