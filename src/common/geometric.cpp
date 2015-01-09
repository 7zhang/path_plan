#ifndef __GEOMETRIC_H_
#define __GEOMETRIC_H_
#include "geometric.h"



////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// 模块描述：Point2D各个友元函数的实现。
// 作者：王成
// 日期：2011-3-4
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
// ----------------------------------------------------------------
Point2D::Point2D(double ix/*=0.0*/,double iy/*=0.0*/)
	:x(ix),y(iy)
{
}
void Point2D::offset(Vector2D vec2)
{
	x += vec2.dx;
	y += vec2.dy;
}
void Point2D::offset( double dx, double dy )
{
	x += dx;
	y += dy;
}
// ----------------------------------------------------------------

bool operator ==( Point2D& lpoint, Point2D& rpoint )
{
	return (isZero(lpoint.x - rpoint.x)
		&& isZero(lpoint.y - rpoint.y));
}
bool operator !=( Point2D& lpoint, Point2D& rpoint )
{
	return !(isZero(lpoint.x - rpoint.x)
		 && isZero(lpoint.y - rpoint.y));
}

// ----------------------------------------------------------------
Point2D operator + ( const Point2D& lpoint, const Point2D& rpoint )
{
	return Point2D( lpoint.x+rpoint.x, lpoint.y+rpoint.y );
}
Point2D operator - ( const Point2D& lpoint, const Point2D& rpoint )
{
	return Point2D( lpoint.x-rpoint.x, lpoint.y-rpoint.y );
}
Point2D& operator += ( Point2D& lpoint, const Point2D& rpoint )
{
	lpoint.x += rpoint.x;
	lpoint.y += rpoint.y;
	return lpoint;
}
Point2D& operator -= ( Point2D& lpoint, const Point2D& rpoint )
{
	lpoint.x -= rpoint.x;
	lpoint.y -= rpoint.y;
	return lpoint;
}


////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// 模块描述：Point3D友元函数等实现。
// 作者：王成
// 日期：2011-3-4
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
Point3D::Point3D(double ix/* =0.0 */, double iy/* =0.0 */, double iz/* =0.0 */)
	:x(ix), y(iy), z(iz)
{
}

void Point3D::offset(Vector3D vec3)
{
	x += vec3.dx;
	y += vec3.dy;
	z += vec3.dz;
}
void Point3D::offset( double dx, double dy, double dz)
{
	x += dx;
	y += dy;
	z += dz;
}
void Point3D::zoom(double rate)
{
	x *= rate;
	y *= rate;
	z *= rate;
}
// ----------------------------------------------------------------

bool operator ==( Point3D& lpoint, Point3D& rpoint )
{
	return (isZero(lpoint.x - rpoint.x)
		&& isZero(lpoint.y - rpoint.y)
		&& isZero(lpoint.z - rpoint.z));
}
bool operator !=( Point3D& lpoint, Point3D& rpoint )
{
	return !(isZero(lpoint.x - rpoint.x)
		 && isZero(lpoint.y - rpoint.y)
		 && isZero(lpoint.z - rpoint.z));
}

// ----------------------------------------------------------------
Point3D operator + ( const Point3D& lpoint, const Vector3D& vec )
{
	return Point3D( lpoint.x+vec.dx, lpoint.y+vec.dy, lpoint.z+vec.dz );
}
Vector3D operator - ( const Point3D& lpoint, const Point3D& rpoint )
{
	return Vector3D( lpoint.x-rpoint.x, lpoint.y-rpoint.y, lpoint.z-rpoint.z );
}
Point3D operator - ( const Point3D& lpoint, const Vector3D& vec )
{
	return Point3D( lpoint.x-vec.dx, lpoint.y-vec.dy, lpoint.z-vec.dz );
}
Point3D& operator += ( Point3D& lpoint, const Vector3D& vec )
{
	lpoint.x += vec.dx;
	lpoint.y += vec.dy;
	lpoint.z += vec.dz;
	return lpoint;
}
Point3D& operator -= ( Point3D& lpoint, const Vector3D& vec )
{
	lpoint.x -= vec.dx;
	lpoint.y -= vec.dy;
	lpoint.z -= vec.dz;
	return lpoint;
}



////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// 模块描述：Vector2D友元函数等实现。
// 作者：王成
// 日期：2011-3-4
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
Vector2D::Vector2D(double ix/* =0.0 */,double iy/* =0.0 */)
	:dx(ix),dy(iy)
{
}
double Vector2D::get_length()
{
	return sqrt(dx*dx + dy*dy);
}

void Vector2D::unitize()
{
//	//assert(!isZero(this->get_length()));
	double len = get_length();
	if (this->get_length() > ZERO)
	{
		dx /= len;
		dy /= len;
	}
}

// ----------------------------------------------------------------
Vector2D operator + ( const Vector2D& lvec, const Vector2D& rvec )
{
	return Vector2D(lvec.dx+rvec.dx, lvec.dy+rvec.dy);
}
Vector2D operator - ( const Vector2D& lvec, const Vector2D& rvec )
{
	return Vector2D(lvec.dx-rvec.dx, lvec.dy-rvec.dy);
}
Vector2D& operator += ( Vector2D& lvec, const Vector2D& rvec )
{
	lvec.dx += rvec.dx;
	lvec.dy += rvec.dy;
	return lvec;
}
Vector2D& operator -= ( Vector2D& lvec, const Vector2D& rvec )
{
	lvec.dx -= rvec.dx;
	lvec.dy -= rvec.dy;
	return lvec;
}

// ----------------------------------------------------------------
Vector2D operator * ( double factor, const Vector2D& vec )
{
	return Vector2D(factor*vec.dx, factor*vec.dy);
}
Vector2D operator * ( const Vector2D& vec, double factor )
{
	return Vector2D(factor*vec.dx, factor*vec.dy);
}
Vector2D& operator *= (Vector2D& vec, double factor)
{
	vec.dx *= factor;
	vec.dy *= factor;
	return vec;
}


Vector2D operator / ( const Vector2D& vec, double factor )
{
	//assert(!isZero(factor));
	return Vector2D(vec.dx/factor, vec.dy/factor);
}

Vector2D& operator /= (Vector2D& vec, double factor)
{
	//assert(!isZero(factor));
	vec.dx /= factor;
	vec.dy /= factor;
	return vec;
}

double operator ^( const Vector2D& lvec, const Vector2D& rvec )
{
	return (lvec.dx*rvec.dx + lvec.dy*rvec.dy );
}

////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// 模块描述：Vector3D友元函数等实现。
// 作者：王成
// 日期：2011-3-4
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
Vector3D::Vector3D( double ix/* =0.0 */, double iy/* =0.0 */, double iz/* =0.0 */ )
	:dx(ix),dy(iy),dz(iz)
{
}

// void Vector3D::offset( double dx, double dy, double dz )
// {
// 	this->dx += dx;
// 	this->dy += dy;
// 	this->dz += dz;
// }
void Vector3D::unitize()
{
	double len = get_length();
	//assert(!isZero(len));
	dx /= len;
	dy /= len;
	dz /= len;
}
double Vector3D::get_length()
{
	return (sqrt(dx*dx+dy*dy+dz*dz));
}
// ----------------------------------------------------------------
Vector3D operator + ( const Vector3D& lvec, const Vector3D& rvec )
{
	return Vector3D(lvec.dx+rvec.dx, lvec.dy+rvec.dy, lvec.dz+rvec.dz);
}
Vector3D operator - ( const Vector3D& lvec, const Vector3D& rvec )
{
	return Vector3D(lvec.dx-rvec.dx, lvec.dy-rvec.dy, lvec.dz-rvec.dz);
}
Vector3D& operator += ( Vector3D& lvec, const Vector3D& rvec )
{
	lvec.dx += rvec.dx;
	lvec.dy += rvec.dy;
	lvec.dz += rvec.dz;
	return lvec;
}
Vector3D& operator -= ( Vector3D& lvec, const Vector3D& rvec )
{
	lvec.dx -= rvec.dx;
	lvec.dy -= rvec.dy;
	lvec.dz -= rvec.dz;
	return lvec;
}

// ----------------------------------------------------------------
Vector3D operator * ( double factor, const Vector3D& vec )
{
	return Vector3D(factor*vec.dx, factor*vec.dy, factor*vec.dz);
}
Vector3D operator * ( const Vector3D& vec, double factor )
{
	return Vector3D(factor*vec.dx, factor*vec.dy, factor*vec.dz);
}
Vector3D& operator *= (Vector3D& vec, double factor)
{
	vec.dx *= factor;
	vec.dy *= factor;
	vec.dz *= factor;
	return vec;
}


Vector3D operator / ( const Vector3D& vec, double factor )
{
	//assert(!isZero(factor));
	return Vector3D(vec.dx/factor, vec.dy/factor, vec.dz/factor);
}

Vector3D& operator /= (Vector3D& vec, double factor)
{
	//assert(!isZero(factor));
	vec.dx /= factor;
	vec.dy /= factor;
	vec.dz /= factor;
	return vec;
}

// -----------------------------叉乘--------------------------------
Vector3D operator * ( const Vector3D& lvec, const Vector3D& rvec )
{
	return Vector3D( 
		lvec.dy*rvec.dz - lvec.dz*rvec.dy,
		-lvec.dx*rvec.dz + lvec.dz*rvec.dx,
		lvec.dx*rvec.dy - lvec.dy*rvec.dx );
}

double operator ^( const Vector3D& lvec, const Vector3D& rvec )
{
	return (lvec.dx*rvec.dx + lvec.dy*rvec.dy +lvec.dz*rvec.dz);
}

////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// 模块描述：Vector4D友元函数等实现。
// 作者：葛浏昊
// 日期：2011-4-16
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
Vector4D::Vector4D( double m1/* =0.0 */, double m2/* =0.0 */, double m3/* =0.0 */, double m4/* =0.0 */ )
{
	mem[0] = m1;
	mem[1] = m2;
	mem[2] = m3;
	mem[3] = m4;
}
// ----------------------------------------------------------------
Vector4D operator + ( const Vector4D& lvec, const Vector4D& rvec )
{
	return Vector4D(lvec.mem[0]+rvec.mem[0], lvec.mem[1]+rvec.mem[1], lvec.mem[2]+rvec.mem[2], lvec.mem[3]+rvec.mem[3]);
}
Vector4D operator - ( const Vector4D& lvec, const Vector4D& rvec )
{
	return Vector4D(lvec.mem[0]-rvec.mem[0], lvec.mem[1]-rvec.mem[1], lvec.mem[2]-rvec.mem[2], lvec.mem[3]-rvec.mem[3]);
}
Vector4D& operator += ( Vector4D& lvec, const Vector4D& rvec )
{
	int i = 0;
	for( i=0; i<4; i++ )
	{
		lvec.mem[i] += rvec.mem[i];
	}
	return lvec;
}
Vector4D& operator -= ( Vector4D& lvec, const Vector4D& rvec )
{
	int i = 0;
	for( i=0; i<4; i++ )
	{
		lvec.mem[i] -= rvec.mem[i];
	}
	return lvec;
}

// ----------------------------------------------------------------
Vector4D operator * ( double factor, const Vector4D& vec )
{
	return Vector4D(factor*vec.mem[0], factor*vec.mem[1], factor*vec.mem[2], factor*vec.mem[3]);
}
Vector4D operator * ( const Vector4D& vec, double factor )
{
	return Vector4D(factor*vec.mem[0], factor*vec.mem[1], factor*vec.mem[2], factor*vec.mem[3]);
}
Vector4D& operator *= (Vector4D& vec, double factor)
{
	int i = 0;
	for( i=0; i<4; i++ )
	{
		vec.mem[i] *= factor;
	}
	return vec;
}


Vector4D operator / ( const Vector4D& vec, double factor )
{
//	//assert(!isZero(factor));
	return Vector4D(vec.mem[0]/factor, vec.mem[1]/factor, vec.mem[2]/factor, vec.mem[3]/factor);
}

Vector4D& operator /= (Vector4D& vec, double factor)
{
//	//assert(!isZero(factor));
	int i = 0;
	for( i=0; i<4; i++ )
	{
		vec.mem[i] /= factor;
	}
	return vec;
}

////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// 模块描述：Matrix3D友元函数等实现。
// 作者：王成
// 日期：2011-3-5
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
Matrix3D::Matrix3D()
{
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			mem[i][j] = (i==j)?1.0:0.0;
		}
}
Matrix3D::Matrix3D(double (&m)[3][3])
{
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			mem[i][j] = m[i][j];
		}
}
Matrix3D::Matrix3D(double (&m)[9])
{
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			mem[i][j] = m[i*3+j];
		}
}
Matrix3D::Matrix3D(const Matrix3D& m)
{
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			mem[i][j] = m.mem[i][j];
		}
}
// ----------------------------------------------------------------
void Matrix3D::transpos()		// 矩阵转置
{
	double temp;
	temp = mem[0][1];
	mem[0][1] = mem[1][0];
	mem[1][0] = temp;
	temp = mem[0][2];
	mem[0][2] = mem[2][0];
	mem[2][0] = temp;
	temp = mem[1][2];
	mem[1][2] = mem[2][1];
	mem[2][1] = temp;
}
void Matrix3D::inverse()			// 矩阵求逆
{
	double deter = get_determinant();
//	//assert(!isZero(deter));
	Matrix3D temp;
	temp.mem[0][0] = (mem[1][1]*mem[2][2] - mem[1][2]*mem[2][1])/deter;
	temp.mem[1][0] = (-mem[1][0]*mem[2][2] + mem[1][2]*mem[2][0])/deter;
	temp.mem[2][0] = (mem[1][0]*mem[2][1] - mem[1][1]*mem[2][0])/deter;

	temp.mem[0][1] = (-mem[0][1]*mem[2][2] + mem[0][2]*mem[2][1])/deter;
	temp.mem[1][1] = (mem[0][0]*mem[2][2] - mem[0][2]*mem[2][0])/deter;
	temp.mem[2][1] = (-mem[0][0]*mem[2][1] + mem[0][1]*mem[2][0])/deter;

	temp.mem[0][2] = (mem[0][1]*mem[1][2] - mem[0][2]*mem[1][1])/deter;
	temp.mem[1][2] = (-mem[0][0]*mem[1][2] + mem[0][2]*mem[1][0])/deter;
	temp.mem[2][2] = (mem[0][0]*mem[1][1] - mem[0][1]*mem[1][0])/deter;

	(*this) = temp;
}
double Matrix3D::get_determinant()	// 行列式值
{
	return ( 
		(mem[0][0]*mem[1][1]*mem[2][2] - mem[0][0]*mem[1][2]*mem[2][1])
		- (mem[0][1]*mem[1][0]*mem[2][2] - mem[0][1]*mem[1][2]*mem[2][0])
		+ (mem[0][2]*mem[1][0]*mem[2][1] - mem[0][2]*mem[1][1]*mem[2][0])
		);
}
void Matrix3D::jacobbi(Matrix3D& v, double* pArray)const
{
	int p , q , j , ind ,n;
	double dsqr , d1 , d2 , thr , dv1 , dv2 , dv3 , dmu , dga , st , ct ;
	double eps = 0.00000001;
	int* iZ; //add 2002.8.27
	
	Matrix3D CA ( *this ) ;
	n = 3 ;
	
	//add 2002.8.27
	iZ = new int[n] ;
	
	for ( p = 0 ; p < n ; p ++ )
		for ( q = 0 ; q < n ; q ++ )
			v.mem[p][q] = ( p == q ) ? 1.0 : 0 ;
		
	dsqr = 0 ;
	for ( p = 1 ; p < n ; p ++ )
		for ( q = 0 ; q < p ; q ++ )
			dsqr += 2 * CA.mem[p][q]* CA.mem[p][q] ;
	d1 = sqrt ( dsqr ) ;
	d2 = eps / n * d1 ;
	thr = d1 ;
	ind = 0 ;
	do {
		thr = thr / n ;
		while ( !ind ) {
			for ( q = 1 ; q < n ; q ++ )
				for ( p = 0 ; p < q ; p ++ )
					if ( fabs ( CA.mem[p][q] ) >= thr ) {
						ind = 1 ;
						dv1 = CA.mem[p][p] ;
						dv2 = CA.mem[p][q] ;
						dv3 = CA.mem[q][q] ;
						dmu = 0.5 * ( dv1 - dv3 ) ;
						double dls = sqrt ( dv2 * dv2 + dmu * dmu ) ;
						if ( fabs ( dmu ) < 0.00000000001 ) dga = -1 ;
						//if ( dmu == 0.0 ) dga = -1.0 ;
						else dga = ( dmu < 0 ) ? ( dv2 / dls ): ( - dv2 / dls );
						st = dga / sqrt ( 2 * ( 1 + sqrt ( 1 - dga * dga ) ) ) ;
						ct = sqrt ( 1 - st * st ) ;
						int l;
						for (  l = 0 ; l < n ; l ++ ) {
							dsqr = CA.mem[l][p] * ct - CA.mem [l][q] * st ;
							CA.mem [l][q] = CA.mem[l][p] * st + CA.mem[l][q] * ct ;
							CA.mem [l][p] = dsqr ;
							dsqr = v.mem[l][p] * ct - v.mem[l][q]*st ;
							v.mem[l][q] = v.mem[l][p] * st + v.mem[l][q] * ct ;
							v.mem[l][p] = dsqr ;
						}
						for ( l = 0 ; l < n ; l ++ ) {
							CA.mem[p][l] = CA.mem[l][p] ;
							CA.mem[q][l] = CA.mem[l][q] ;
						}
						CA.mem [p][p] = dv1 * ct * ct + dv3 * st * st - 2 * dv2 * st * ct;
						CA.mem [q][q] = dv1 * st * st + dv3 * ct * ct + 2 * dv2 * st * ct;
						CA.mem [p][q] = CA.mem[q][p] = 0.0 ;
					}
			if ( ind ) ind = 0 ;
			else break ;
		}
	} while ( thr > d2 ) ;
	for ( int l = 0 ; l < n ; l ++ ) {
		pArray[l] = CA.mem[l][l];
		iZ [l] = l;
	}
	//2002.8.27 add
	double dTemp ;
	int i , k;
			
	for ( i = 0 ; i < n ; i++ ){
		//dmax = pArray[i];
		for ( j = i + 1 ; j < n ; j++ ){
			if ( pArray[i] < pArray[j]){
				dTemp = pArray[i];
				pArray[i] = pArray[j];
				pArray[j] = dTemp;
				k = iZ[i] ;
				iZ[i] = iZ[j];
				iZ[j] = k;
			}
		}
	}
	CA = v;
			
	for ( j = 0 ; j < n ; j++ )
		for ( i = 0 ; i < n ; i++ )
			v.mem [i][j] = CA.mem[i][iZ[j]];
				
	delete[] iZ;
	//2002.8.27 end add
}
// ----------------------------------------------------------------
Matrix3D operator+( const Matrix3D& lma, const Matrix3D& rma )
{
	Matrix3D temp;

	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			temp.mem[i][j] = lma.mem[i][j] + rma.mem[i][j];
		}
	return temp;
}
Matrix3D operator-( const Matrix3D& lma, const Matrix3D& rma )
{
	Matrix3D temp;
	
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			temp.mem[i][j] = lma.mem[i][j] - rma.mem[i][j];
		}
	return temp;
}
Matrix3D& operator+=( Matrix3D& lma, const Matrix3D& rma )
{
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			lma.mem[i][j] += rma.mem[i][j];
		}
	return lma;
}
Matrix3D& operator-=( Matrix3D& lma, const Matrix3D& rma )
{
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			lma.mem[i][j] -= rma.mem[i][j];
		}
	return lma;
}
Matrix3D operator*( const Matrix3D& lma, const Matrix3D& rma )
{
	Matrix3D matrix;
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++){
			matrix.mem[i][j] = 
				lma.mem[i][0]*rma.mem[0][j]
				+ lma.mem[i][1]*rma.mem[1][j]
				+ lma.mem[i][2]*rma.mem[2][j];
		}
	return matrix;
}
Matrix3D& operator*=( Matrix3D& lma, const Matrix3D& rma )
{
	Matrix3D matrix(lma);
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++){
			lma.mem[i][j] = 
				matrix.mem[i][0]*rma.mem[0][j]
				+ matrix.mem[i][1]*rma.mem[1][j]
				+ matrix.mem[i][2]*rma.mem[2][j];
		}
	return lma;
}
Vector3D operator*( const Matrix3D& lma, const Vector3D& rvec)
{
	
	return Vector3D(
		lma.mem[0][0]*rvec.dx + lma.mem[0][1]*rvec.dy + lma.mem[0][2]*rvec.dz,
		lma.mem[1][0]*rvec.dx + lma.mem[1][1]*rvec.dy + lma.mem[1][2]*rvec.dz,
		lma.mem[2][0]*rvec.dx + lma.mem[2][1]*rvec.dy + lma.mem[2][2]*rvec.dz
		);
}

// ----------------------------------------------------------------
Matrix3D operator*(double factor, const Matrix3D& ma)
{
	double a[9] = {
		factor*ma.mem[0][0],factor*ma.mem[0][1],factor*ma.mem[0][2],
		factor*ma.mem[1][0],factor*ma.mem[1][1],factor*ma.mem[1][2],
		factor*ma.mem[2][0],factor*ma.mem[2][1],factor*ma.mem[2][2]};
	return Matrix3D(a);
}
Matrix3D operator*(const Matrix3D& ma, double factor)
{
	double a[9] = {
		factor*ma.mem[0][0],factor*ma.mem[0][1],factor*ma.mem[0][2],
		factor*ma.mem[1][0],factor*ma.mem[1][1],factor*ma.mem[1][2],
		factor*ma.mem[2][0],factor*ma.mem[2][1],factor*ma.mem[2][2]};
	return Matrix3D(a);
}
Matrix3D& operator*=(Matrix3D& ma, double factor)
{
	ma.mem[0][0] *= factor;
	ma.mem[0][1] *= factor;
	ma.mem[0][2] *= factor;
	ma.mem[1][0] *= factor;
	ma.mem[1][1] *= factor;
	ma.mem[1][2] *= factor;
	ma.mem[2][0] *= factor;
	ma.mem[2][1] *= factor;
	ma.mem[2][2] *= factor;
	return ma;
}

Matrix3D operator/(const Matrix3D& ma, double factor)
{
//	//assert(!isZero(factor));
	double a[9] = {
		ma.mem[0][0]/factor, ma.mem[0][1]/factor, ma.mem[0][2]/factor,
		ma.mem[1][0]/factor, ma.mem[1][1]/factor, ma.mem[1][2]/factor,
		ma.mem[2][0]/factor, ma.mem[2][1]/factor, ma.mem[2][2]/factor};
	return Matrix3D(a);
}
Matrix3D& operator/=(Matrix3D& ma, double factor)
{
//	//assert(!isZero(factor));
	ma.mem[0][0] /= factor;
	ma.mem[0][1] /= factor;
	ma.mem[0][2] /= factor;
	ma.mem[1][0] /= factor;
	ma.mem[1][1] /= factor;
	ma.mem[1][2] /= factor;
	ma.mem[2][0] /= factor;
	ma.mem[2][1] /= factor;
	ma.mem[2][2] /= factor;
	return ma;
}

////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// 模块描述：Matrix4D友元函数等实现。
// 作者：葛浏昊
// 日期：2011-3-17
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
Matrix4D::Matrix4D()
{
	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
		{
			mem[i][j] = (i==j)?1.0:0.0;
		}
//	rot=get_rot(mem);
//	pos=get_pos(mem);
}
Matrix4D::Matrix4D(double (&m)[4][4])
{
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			mem[i][j] = m[i][j];
		}
	}
//	rot=get_rot(mem);
//	pos=get_pos(mem);
}
Matrix4D::Matrix4D(const Matrix3D& Rot,const Vector3D& Pos)	//齐次变换矩阵
{
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			mem[i][j] = Rot.mem[i][j];
		}
	mem[0][3]=Pos.dx; mem[1][3]=Pos.dy; mem[2][3]=Pos.dz;
	for(int j=0;j<3;j++)
	{
		mem[3][j] =0;
	}
	mem[3][3]=1;
//	rot=Rot;
//	pos=Pos;
}
Matrix4D::Matrix4D(const Matrix4D& m)
{
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			mem[i][j] = m.mem[i][j];
		}
	}
//	rot=m.rot;
//	pos=m.pos;
}
// ----------------------------------------------------------------
void Matrix4D::transpos()		// 矩阵转置
{
	double temp;
	for (int i=0;i<4;i++)
	{
		for (int j=i+1;j<4;j++)
		{
			temp = mem[i][j];
			mem[i][j] = mem[j][i];
			mem[j][i] = temp;
		}
	}
//	rot=get_rot(mem);
//	pos=get_pos(mem);
}
void Matrix4D::inverse()			// 矩阵求逆
{
	double deter = get_determinant();
//	//assert(!isZero(deter));
	Matrix4D temp;

	Matrix3D temp00(mem[1][1],mem[1][2],mem[1][3],mem[2][1],mem[2][2],mem[2][3],mem[3][1],mem[3][2],mem[3][3]);
	temp.mem[0][0] = temp00.get_determinant()/deter;
	Matrix3D temp10(mem[1][0],mem[1][2],mem[1][3],mem[2][0],mem[2][2],mem[2][3],mem[3][0],mem[3][2],mem[3][3]);
	temp.mem[1][0] = -temp10.get_determinant()/deter;
	Matrix3D temp20(mem[1][0],mem[1][1],mem[1][3],mem[2][0],mem[2][1],mem[2][3],mem[3][0],mem[3][1],mem[3][3]);
	temp.mem[2][0] = temp20.get_determinant()/deter;
	Matrix3D temp30(mem[1][0],mem[1][1],mem[1][2],mem[2][0],mem[2][1],mem[2][2],mem[3][0],mem[3][1],mem[3][2]);
	temp.mem[3][0] = -temp30.get_determinant()/deter;

	Matrix3D temp01(mem[0][1],mem[0][2],mem[0][3],mem[2][1],mem[2][2],mem[2][3],mem[3][1],mem[3][2],mem[3][3]);
	temp.mem[0][1] = -temp01.get_determinant()/deter;
	Matrix3D temp11(mem[0][0],mem[0][2],mem[0][3],mem[2][0],mem[2][2],mem[2][3],mem[3][0],mem[3][2],mem[3][3]);
	temp.mem[1][1] = temp11.get_determinant()/deter;
	Matrix3D temp21(mem[0][0],mem[0][1],mem[0][3],mem[2][0],mem[2][1],mem[2][3],mem[3][0],mem[3][1],mem[3][3]);
	temp.mem[2][1] = -temp21.get_determinant()/deter;
	Matrix3D temp31(mem[0][0],mem[0][1],mem[0][2],mem[2][0],mem[2][1],mem[2][2],mem[3][0],mem[3][1],mem[3][2]);
	temp.mem[3][1] = temp31.get_determinant()/deter;

	Matrix3D temp02(mem[0][1],mem[0][2],mem[0][3],mem[1][1],mem[1][2],mem[1][3],mem[3][1],mem[3][2],mem[3][3]);
	temp.mem[0][2] = temp02.get_determinant()/deter;
	Matrix3D temp12(mem[0][0],mem[0][2],mem[0][3],mem[1][0],mem[1][2],mem[1][3],mem[3][0],mem[3][2],mem[3][3]);
	temp.mem[1][2] = -temp12.get_determinant()/deter;
	Matrix3D temp22(mem[0][0],mem[0][1],mem[0][3],mem[1][0],mem[1][1],mem[1][3],mem[3][0],mem[3][1],mem[3][3]);
	temp.mem[2][2] = temp22.get_determinant()/deter;
	Matrix3D temp32(mem[0][0],mem[0][1],mem[0][2],mem[1][0],mem[1][1],mem[1][2],mem[3][0],mem[3][1],mem[3][2]);
	temp.mem[3][2] = -temp32.get_determinant()/deter;

	Matrix3D temp03(mem[0][1],mem[0][2],mem[0][3],mem[1][1],mem[1][2],mem[1][3],mem[2][1],mem[2][2],mem[2][3]);
	temp.mem[0][3] = -temp03.get_determinant()/deter;
	Matrix3D temp13(mem[0][0],mem[0][2],mem[0][3],mem[1][0],mem[1][2],mem[1][3],mem[2][0],mem[2][2],mem[2][3]);
	temp.mem[1][3] = temp13.get_determinant()/deter;
	Matrix3D temp23(mem[0][0],mem[0][1],mem[0][3],mem[1][0],mem[1][1],mem[1][3],mem[2][0],mem[2][1],mem[2][3]);
	temp.mem[2][3] = -temp23.get_determinant()/deter;
	Matrix3D temp33(mem[0][0],mem[0][1],mem[0][2],mem[1][0],mem[1][1],mem[1][2],mem[2][0],mem[2][1],mem[2][2]);
	temp.mem[3][3] = temp33.get_determinant()/deter;

//	temp.mem[0][3] = -(temp.mem[0][0]*mem[0][3]+temp.mem[0][1]*mem[1][3]+temp.mem[0][2]*mem[2][3]);
//	temp.mem[1][3] = -(temp.mem[1][0]*mem[0][3]+temp.mem[1][1]*mem[1][3]+temp.mem[1][2]*mem[2][3]);
//	temp.mem[2][3] = -(temp.mem[2][0]*mem[0][3]+temp.mem[2][1]*mem[1][3]+temp.mem[2][2]*mem[2][3]);

//	temp.mem[3][0]=temp.mem[3][1]=temp.mem[3][2]=0;
//	temp.mem[3][3]=1;

//	temp.rot=get_rot(temp.mem);
//	temp.pos=get_pos(temp.mem);

	(*this) = temp;
}
double Matrix4D::get_determinant()	// 行列式值
{ 
	return ( 
		(  (mem[0][0]*mem[1][1]*mem[2][2] - mem[0][0]*mem[1][2]*mem[2][1])
		   - (mem[0][1]*mem[1][0]*mem[2][2] - mem[0][1]*mem[1][2]*mem[2][0])
		   + (mem[0][2]*mem[1][0]*mem[2][1] - mem[0][2]*mem[1][1]*mem[2][0]) )*mem[3][3]
		-(  (mem[0][0]*mem[1][1]*mem[2][3] - mem[0][0]*mem[1][3]*mem[2][1])
		    - (mem[0][1]*mem[1][0]*mem[2][3] - mem[0][1]*mem[1][3]*mem[2][0])
		    + (mem[0][3]*mem[1][0]*mem[2][1] - mem[0][3]*mem[1][1]*mem[2][0]) )*mem[3][2]
		+(  (mem[0][0]*mem[1][2]*mem[2][3] - mem[0][0]*mem[1][3]*mem[2][2])
		    - (mem[0][2]*mem[1][0]*mem[2][3] - mem[0][2]*mem[1][3]*mem[2][0])
		    + (mem[0][3]*mem[1][0]*mem[2][2] - mem[0][3]*mem[1][2]*mem[2][0]) )*mem[3][1]
		-(  (mem[0][1]*mem[1][2]*mem[2][3] - mem[0][1]*mem[1][3]*mem[2][2])
		    - (mem[0][2]*mem[1][1]*mem[2][3] - mem[0][2]*mem[1][3]*mem[2][1])
		    + (mem[0][3]*mem[1][1]*mem[2][2] - mem[0][3]*mem[1][2]*mem[2][1]) )*mem[3][0]
		);
}
void Matrix4D::jacobbi(Matrix4D& v, double* pArray)const
{
	int p , q , j , ind ,n;
	double dsqr , d1 , d2 , thr , dv1 , dv2 , dv3 , dmu , dga , st , ct ;
	double eps = 0.00000001;
	int* iZ; //add 2002.8.27
	
	Matrix4D CA ( *this ) ; 
	n = 4 ;
	
	//add 2002.8.27
	iZ = new int[n] ;


	
	for ( p = 0 ; p < n ; p ++ )
		for ( q = 0 ; q < n ; q ++ )
			v.mem[p][q] = ( p == q ) ? 1.0 : 0 ;
		
	dsqr = 0 ;
	for ( p = 1 ; p < n ; p ++ )
		for ( q = 0 ; q < p ; q ++ )
			dsqr += 2 * CA.mem[p][q]* CA.mem[p][q] ;
	d1 = sqrt ( dsqr ) ;
	d2 = eps / n * d1 ;
	thr = d1 ;
	ind = 0 ;
	do {
		thr = thr / n ;
		while ( !ind ) {
			for ( q = 1 ; q < n ; q ++ )
				for ( p = 0 ; p < q ; p ++ )
					if ( fabs ( CA.mem[p][q] ) >= thr ) {
						ind = 1 ;
						dv1 = CA.mem[p][p] ;
						dv2 = CA.mem[p][q] ;
						dv3 = CA.mem[q][q] ;
						dmu = 0.5 * ( dv1 - dv3 ) ;
						double dls = sqrt ( dv2 * dv2 + dmu * dmu ) ;
						if ( fabs ( dmu ) < 0.00000000001 ) dga = -1 ;
						//if ( dmu == 0.0 ) dga = -1.0 ;
						else dga = ( dmu < 0 ) ? ( dv2 / dls ): ( - dv2 / dls );
						st = dga / sqrt ( 2 * ( 1 + sqrt ( 1 - dga * dga ) ) ) ;
						ct = sqrt ( 1 - st * st ) ;
						int l;
						for (  l = 0 ; l < n ; l ++ ) {
							dsqr = CA.mem[l][p] * ct - CA.mem [l][q] * st ;
							CA.mem [l][q] = CA.mem[l][p] * st + CA.mem[l][q] * ct ;
							CA.mem [l][p] = dsqr ;
							dsqr = v.mem[l][p] * ct - v.mem[l][q]*st ;
							v.mem[l][q] = v.mem[l][p] * st + v.mem[l][q] * ct ;
							v.mem[l][p] = dsqr ;
						}
						for ( l = 0 ; l < n ; l ++ ) {
							CA.mem[p][l] = CA.mem[l][p] ;
							CA.mem[q][l] = CA.mem[l][q] ;
						}
						CA.mem [p][p] = dv1 * ct * ct + dv3 * st * st - 2 * dv2 * st * ct;
						CA.mem [q][q] = dv1 * st * st + dv3 * ct * ct + 2 * dv2 * st * ct;
						CA.mem [p][q] = CA.mem[q][p] = 0.0 ;
					}
			if ( ind ) ind = 0 ;
			else break ;
		}
	} while ( thr > d2 ) ;
	for ( int l = 0 ; l < n ; l ++ ) {
		pArray[l] = CA.mem[l][l];
		iZ [l] = l;
	}
	//2002.8.27 add
	double dTemp ;
	int i , k;
			
	for ( i = 0 ; i < n ; i++ ){
		//dmax = pArray[i];
		for ( j = i + 1 ; j < n ; j++ ){
			if ( pArray[i] < pArray[j]){
				dTemp = pArray[i];
				pArray[i] = pArray[j];
				pArray[j] = dTemp;
				k = iZ[i] ;
				iZ[i] = iZ[j];
				iZ[j] = k;
			}
		}
	}
	CA = v;
			
	for ( j = 0 ; j < n ; j++ )
		for ( i = 0 ; i < n ; i++ )
			v.mem [i][j] = CA.mem[i][iZ[j]];
				
	delete[] iZ;
	//2002.8.27 end add
}
// ----------------------------------------------------------------
Matrix4D operator*( const Matrix4D& lma, const Matrix4D& rma )
{
	Matrix4D matrix;
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			matrix.mem[i][j] = 
				lma.mem[i][0]*rma.mem[0][j]
				+ lma.mem[i][1]*rma.mem[1][j]
				+ lma.mem[i][2]*rma.mem[2][j]
				+ lma.mem[i][3]*rma.mem[3][j];
		}
	}
//	matrix.rot=matrix.get_rot(matrix.mem);
//	matrix.pos=matrix.get_pos(matrix.mem);
	return matrix;
}

Matrix4D& operator*=( Matrix4D& lma, const Matrix4D& rma )
{
	Matrix4D matrix(lma);
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			lma.mem[i][j] = 
				matrix.mem[i][0]*rma.mem[0][j]
				+ matrix.mem[i][1]*rma.mem[1][j]
				+ matrix.mem[i][2]*rma.mem[2][j]
				+ matrix.mem[i][3]*rma.mem[3][j];
		}
	}
//	lma.rot=lma.get_rot(lma.mem);
//	lma.pos=lma.get_pos(lma.mem);
	return lma;
}

Vector4D operator*( const Matrix4D& lma, const Vector4D& rvec)
{
	int i,j;
	Vector4D res(0,0,0,0);
	for ( i=0; i<4; i++ )
	{
		for ( j=0; j<4; j++ )
		{
			res.mem[i] += lma.mem[i][j]*rvec.mem[j];
		}
	}
	return res;
}

Vector4D& operator*=( Vector4D& vec, const Matrix4D& matrix)
{
	vec = matrix * vec;
	return vec;
}

Point3D operator*(const Matrix4D& matrix, const Point3D& point)
{
	Vector4D v(point.x, point.y, point.z, 1.0);
	v = matrix * v;
	return Point3D(v.mem[0], v.mem[1], v.mem[2]);
}

Point3D& operator*=( Point3D& point, const Matrix4D& matrix)
{
	point = matrix * point;
	return point;
}

Vector3D operator*(const Matrix4D& matrix, const Vector3D& vec)
{
	Vector4D v(vec.dx, vec.dy, vec.dz, 1.0);
	v = matrix * v;
	return Vector3D(v.mem[0], v.mem[1], v.mem[2]);
}

Vector3D& operator*=(Vector3D& vec , const Matrix4D& matrix)
{
	vec = matrix * vec;
	return vec;
}

#endif
