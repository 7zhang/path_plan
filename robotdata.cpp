#define __ROBOTDATA_EXP_
#include "robotdata.h"

////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// 模块描述：JAngle各个友元函数的实现。
// 作者：王成
// 日期：2011-3-4
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
JAngle::JAngle(double a1, double a2, double a3, double a4, double a5, double a6)
{
	angle[0]=a1;
	angle[1]=a2;
	angle[2]=a3;
	angle[3]=a4;
	angle[4]=a5;
	angle[5]=a6;
}

JAngle::JAngle(double a[])
{
	angle[0] = a[0];
	angle[1] = a[1];
	angle[2] = a[2];
	angle[3] = a[3];
	angle[4] = a[4];
	angle[5] = a[5];
}

void JAngle::set_angle(double value, int num){	
//		assert( (num>0 && num <7) );
	//assert( (pos>-1 && pos <6) );  
	angle[num-1] = value;           //修改
}

void JAngle::set_angles(double ang1, double ang2, double ang3, double ang4, double ang5, double ang6)
{
	angle[0] = ang1;
	angle[1] = ang2;
	angle[2] = ang3;
	angle[3] = ang4;
	angle[4] = ang5;
	angle[5] = ang6;
}

double JAngle::get_angle(int num) const
{
//		assert( (num>0 && num <7) );
	//assert( (pos>-1 && pos <6) );  
	return angle[num-1];           //修改
}		



bool JAngle::operator==(const JAngle& rhs)const
{
	if ( fabs(angle[0]-rhs.angle[0]) < 0.001 &&
	     fabs(angle[1]-rhs.angle[1]) < 0.001 &&
	     fabs(angle[2]-rhs.angle[2]) < 0.001 &&
	     fabs(angle[3]-rhs.angle[3]) < 0.001 &&
	     fabs(angle[4]-rhs.angle[4]) < 0.001 &&
	     fabs(angle[5]-rhs.angle[5]) < 0.001)
	{
		return true;
	}
	return false;
}

bool JAngle::operator!=(const JAngle& rhs)const
{
	return !(*this == rhs);
}


////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// 模块描述：TRANS各个友元函数的实现。
// 作者：葛浏昊
// 日期：2011-3-18
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
void TRANS::trans(double px, double py, double pz)	// 标架平移
{
	pos.dx += px;
	pos.dy += py;
	pos.dz += pz;
}

void TRANS::rotx(double angle)						// 标架旋转
{
	//added by WangHaibo@2012
	//右乘
	angle = angle*PI/180.0;//把角度转化成弧度
	TRANS rotateX(1.0, 0.0, 0.0,
		      0.0, cos(angle), sin(angle),
		      0.0, -sin(angle), cos(angle),
		      0.0, 0.0, 0.0               
		);
	*this *= rotateX;
}

void TRANS::roty(double angle)						// 标架旋转
{
	//added by WangHaibo@2012
	//右乘
	angle = angle*PI/180.0;//把角度转化成弧度
	TRANS rotateY(cos(angle), 0.0, -sin(angle),
		      0.0, 1.0, 0.0,
		      sin(angle), 0.0, cos(angle),
		      0.0, 0.0, 0.0           
		);
	*this *= rotateY;
}

void TRANS::rotz(double angle)						// 标架旋转
{
	//added by WangHaibo@2012
	//右乘
	angle = angle*PI/180.0;//把角度转化成弧度
	TRANS rotateZ(cos(angle), sin(angle), 0.0,
		      -sin(angle), cos(angle), 0.0,
		      0.0, 0.0, 1.0,
		      0.0, 0.0, 0.0    
		);
	*this *= rotateZ;
}

void TRANS::inverse()
{
	rot.inverse();
	Vector3D temp;
	temp.dx = -(rot.mem[0][0]*pos.dx + rot.mem[0][1]*pos.dy + rot.mem[0][2]*pos.dz);
	temp.dy = -(rot.mem[1][0]*pos.dx + rot.mem[1][1]*pos.dy + rot.mem[1][2]*pos.dz);
	temp.dz = -(rot.mem[2][0]*pos.dx + rot.mem[2][1]*pos.dy + rot.mem[2][2]*pos.dz);
	pos = temp;
}

bool TRANS::Trans2Euler(Euler& Eu,const Euler& lastEu) const
{
	Eu.pos = pos;
	double fi,theta,pusi;

	if (fabs(rot.mem[1][2])<1e-4 && fabs(rot.mem[0][2])<1e-4)   //sin(theta)=0
	{
		if ((fabs(rot.mem[2][2])-1)<1e-4)
		{
			theta=0;
		}
		else
		{
			theta=PI;
		}
		fi=lastEu.orient.dx*PI/180;       //此时只能求出fi,pusi的和/差，故这里选定fi与上一时刻相同
	}
	else
	{
		fi = atan2(rot.mem[1][2],rot.mem[0][2]);      //fi
		theta = atan2(cos(fi)*rot.mem[0][2]+sin(fi)*rot.mem[1][2],rot.mem[2][2]);  //theta
	}
	pusi = atan2(-sin(fi)*rot.mem[0][0]+cos(fi)*rot.mem[1][0],-sin(fi)*rot.mem[0][1]+cos(fi)*rot.mem[1][1]);//pusi

	Eu.orient.dx = fi*180/PI;
	Eu.orient.dy = theta*180/PI;
	Eu.orient.dz = pusi*180/PI;

	return true;
}//End T6ToAuxiT6()


bool TRANS::Trans2Euler(Euler& Eu) const
{
	Eu.pos = pos;
	double fi,theta,pusi;
	 
	if ((pow(rot.mem[2][0], 2) + pow(rot.mem[2][1], 2))<1e-4)   //sin(theta)=0
	{
		if (rot.mem[0][0]<0)
		{
			fi=0;
			theta=PI;
			pusi=atan2(rot.mem[0][1], -rot.mem[0][0]);
		}
		else
		{
			fi=0;
			theta=0;
			pusi=atan2(-rot.mem[0][1], rot.mem[0][0]);
			 
		}
		 
	}
	else
	{
		fi=atan2(rot.mem[1][2], rot.mem[0][2]);
		theta=atan2(sqrt((pow(rot.mem[2][0], 2) + pow(rot.mem[2][1], 2))), rot.mem[2][2]);
		pusi=atan2(rot.mem[2][1], -rot.mem[2][0]);
	}	 
	Eu.orient.dx = fi*180/PI;
	Eu.orient.dy = theta*180/PI;
	Eu.orient.dz = pusi*180/PI;
	 
	return true;
}


bool TRANS::Trans2RPY(RPY& rpy,const RPY& lastRPY) const
{
	rpy.pos=pos;
	double pusi,theta,fi;
//	double ctheta;
	if (fabs(rot.mem[1][0])<1e-4 && fabs(rot.mem[0][0])<1e-4)
	{
		/*	if ((fabs(rot.mem[2][0])-1)<1e-5)
			{
			theta=-PI/2;
			}
			else
			{
			theta=PI/2;
			}*/
		theta = atan2(-rot.mem[2][0],0);
		fi = lastRPY.orient.dz*PI/180;    //此时只能求出fi,pusi的和/差，故这里选定fi与上一时刻相同
		//	pusi = atan2(sin(fi)*rot.mem[0][2]-cos(fi)*rot.mem[1][2],cos(fi)*rot.mem[1][1]-sin(fi)*rot.mem[0][1]);
	}
	else
	{
		fi = atan2(rot.mem[1][0],rot.mem[0][0]);      
		//	pusi = atan2(rot.mem[2][1],rot.mem[2][2]);
		theta = atan2(-rot.mem[2][0],cos(fi)*rot.mem[0][0]+sin(fi)*rot.mem[1][0]);
		if( cos(theta)<0 )
		{
			fi = atan2(-rot.mem[1][0],-rot.mem[0][0]);      
			//	pusi = atan2(rot.mem[2][1],rot.mem[2][2]);
			theta = atan2(-rot.mem[2][0],cos(fi)*rot.mem[0][0]+sin(fi)*rot.mem[1][0]);
			if( cos(theta)>0 )
			{
				return false;
			}
		}

		/*	if(fabs(rot.mem[2][1])<1e-5)
			ctheta=rot.mem[2][2]/cos(pusi);
			else
			ctheta=rot.mem[2][1]/sin(pusi);

			theta=atan2(-rot.mem[2][0],ctheta);*/
	}

	double spusi = sin(fi)*rot.mem[0][2]-cos(fi)*rot.mem[1][2];
	double cpusi = cos(fi)*rot.mem[1][1]-sin(fi)*rot.mem[0][1];
	if( fabs(spusi) < 1e-4 )
	{
		spusi = 0;			//为防止出现180和-180的重解问题
	}
	pusi = atan2(spusi,cpusi);

	rpy.orient.dx=pusi*180/PI;         
	rpy.orient.dy=theta*180/PI;
	rpy.orient.dz=fi*180/PI;
	
	return true;
}


bool TRANS::Trans2RPY(RPY &rpy) const
{
	rpy.pos=pos;
	double pusi/*dx*/,theta/*dy*/,fi/*dz*/;
	if (fabs(rot.mem[1][0])<1e-4 && fabs(rot.mem[0][0])<1e-4)	//theta=(+/-)90
	{
		if ( rot.mem[2][0]<0 )
		{
			theta = PI/2;
			fi = 0;
			pusi = atan2( rot.mem[0][1], rot.mem[1][1] );
		}
		else if ( rot.mem[2][0]>0 )
		{
			theta = -PI/2;
			fi = 0;
			pusi = -atan2( rot.mem[0][1], rot.mem[1][1] );
		}
		else
		{
			return false;
		}
	}
	else
	{
		theta = atan2( -rot.mem[2][0], sqrt( pow(rot.mem[0][0],2)+pow(rot.mem[1][0],2) ) );
		double ctheta = cos( theta );
		if ( fabs(ctheta)<1e-4 )
		{
			return false;
		}
		else
		{
			fi = atan2( rot.mem[1][0]/ctheta, rot.mem[0][0]/ctheta );
			pusi = atan2( rot.mem[2][1]/ctheta, rot.mem[2][2]/ctheta );
		}
	}
	
	//单位转换,弧度->度
	rpy.orient.dx=pusi*180/PI;         
	rpy.orient.dy=theta*180/PI;
	rpy.orient.dz=fi*180/PI;

	
	return true;
}

bool TRANS::Trans2Quat( Quaternion& quat ) const
{
	double qs = sqrt(rot.mem[0][0]+rot.mem[1][1]+rot.mem[2][2]+1)/2.0;
	double kx = rot.mem[2][1] - rot.mem[1][2];	// Oz - Ay
	double ky = rot.mem[0][2] - rot.mem[2][0];	// Ax - Nz
	double kz = rot.mem[1][0] - rot.mem[0][1];	// Ny - Ox
	double kx1, ky1, kz1;
	bool add = false;
		
	if ( (rot.mem[0][0] >= rot.mem[1][1]) && (rot.mem[0][0] >= rot.mem[2][2]) )
	{
		kx1 = rot.mem[0][0] - rot.mem[1][1] - rot.mem[2][2] +1;	// Nx - Oy - Az + 1
		ky1 = rot.mem[1][0] + rot.mem[0][1];	// Ny + Ox
		kz1 = rot.mem[2][0] + rot.mem[0][2];	// Nz + Ax
		add = (kx >= 0);
	}
	else if ( (rot.mem[1][1] >= rot.mem[2][2]) )
	{
		kx1 = rot.mem[1][0] + rot.mem[0][1];	// Ny + Ox
		ky1 = rot.mem[1][1] - rot.mem[0][0] - rot.mem[2][2] +1;	// Oy - Nx - Az + 1
		kz1 = rot.mem[2][1] + rot.mem[1][2];	// Oz + Ay
		add = (ky >= 0);
	}
	else
	{
		kx1 = rot.mem[2][0] + rot.mem[0][2];	// Nz + Ax
		ky1 = rot.mem[2][1] + rot.mem[1][2];	// Oz + Ay
		kz1 = rot.mem[2][2] - rot.mem[0][0] - rot.mem[1][1] +1;	// Az - Nx - Oy + 1
		add = (kz >= 0);
	}

	if (add)
	{
		kx = kx + kx1;
		ky = ky + ky1;
		kz = kz + kz1;
	}
	else
	{
		kx = kx - kx1;
		ky = ky - ky1;
		kz = kz - kz1;
	}
			
	double nm = sqrt(kx*kx+ky*ky+kz*kz);
	if (isZero(nm))
	{
		quat = Quaternion(1,0,0,0);
	}
	else
	{
		double s = sqrt(1 - qs*qs) / nm;
		quat = Quaternion(qs, s*kx, s*ky, s*kz);
	}

	return true;
}

Euler TRANS::GetEuler() const
{
	Euler Eu;
	Eu.pos = pos;
	double fi,theta,pusi;
	
	if ((pow(rot.mem[2][0], 2) + pow(rot.mem[2][1], 2))<1e-4)   //sin(theta)=0
	{
		if (rot.mem[0][0]<0)
		{
			fi=0;
			theta=PI;
			pusi=atan2(rot.mem[0][1], -rot.mem[0][0]);
		}
		else
		{
			fi=0;
			theta=0;
			pusi=atan2(-rot.mem[0][1], rot.mem[0][0]);
			
		}	
	}
	else
	{
		fi=atan2(rot.mem[1][2], rot.mem[0][2]);
		theta=atan2((pow(rot.mem[2][0], 2) + pow(rot.mem[2][1], 2)), rot.mem[2][2]);
		pusi=atan2(rot.mem[2][1], -rot.mem[2][0]);
	}	 
	Eu.orient.dx = fi*180/PI;
	Eu.orient.dy = theta*180/PI;
	Eu.orient.dz = pusi*180/PI;	
	
	return Eu;
}


RPY TRANS::GetRPY() const 
{
	return false;
	RPY rpy;
	rpy.pos=pos;
	double pusi,theta,fi;
	if (fabs(rot.mem[1][0])<1e-4 && fabs(rot.mem[0][0])<1e-4)	//theta=(+/-)90
	{
		if ( rot.mem[2][0]<0 )
		{
			theta = PI/2;
			fi = 0;
			pusi = atan2( rot.mem[0][1], rot.mem[1][1] );
		}
		else if ( rot.mem[2][0]>0 )
		{
			theta = -PI/2;
			fi = 0;
			pusi = -atan2( rot.mem[0][1], rot.mem[1][1] );
		}
		else
		{
			return false;
		}
	}
	else
	{
		theta = atan2( -rot.mem[2][0], sqrt( pow(rot.mem[0][0],2)+pow(rot.mem[1][0],2) ) );
		double ctheta = cos( theta );
		if ( fabs(ctheta)<1e-4 )
		{
			return false;
		}
		else
		{
			fi = atan2( rot.mem[1][0]/ctheta, rot.mem[0][0]/ctheta );
			pusi = atan2( rot.mem[2][1]/ctheta, rot.mem[2][2]/ctheta );
		}
	}
	
	//单位转换,弧度->度
	rpy.orient.dx=pusi*180/PI;         
	rpy.orient.dy=theta*180/PI;
	rpy.orient.dz=fi*180/PI;
	
	return rpy;	
}


Vector3D TRANS::operator *(const Vector3D& rvec) const		// 坐标变换
{
	Vector3D eVec(
		rot.mem[0][0]*rvec.dx + rot.mem[0][1]*rvec.dy + rot.mem[0][2]*rvec.dz + pos.dx,
		rot.mem[1][0]*rvec.dx + rot.mem[1][1]*rvec.dy + rot.mem[1][2]*rvec.dz + pos.dy,
		rot.mem[2][0]*rvec.dx + rot.mem[2][1]*rvec.dy + rot.mem[2][2]*rvec.dz + pos.dz
		);
	Vector3D sVec(pos.dx,pos.dy,pos.dz);
	return (eVec-sVec);
}

Point3D TRANS::operator *(const Point3D& rvec)	const	// 坐标变换
{
	return Point3D(
		rot.mem[0][0]*rvec.x + rot.mem[0][1]*rvec.y + rot.mem[0][2]*rvec.z + pos.dx,
		rot.mem[1][0]*rvec.x + rot.mem[1][1]*rvec.y + rot.mem[1][2]*rvec.z + pos.dy,
		rot.mem[2][0]*rvec.x + rot.mem[2][1]*rvec.y + rot.mem[2][2]*rvec.z + pos.dz
		);
}

TRANS TRANS::operator * (const TRANS& t6) const			// 标架乘法
{
	Matrix3D Rot = rot*t6.rot;
	Vector3D Pos = rot*t6.pos + pos;

	TRANS t(Rot, Pos);
	return t;
}

void TRANS::operator *= (const TRANS& t6)
{
	(*this) = (*this)*t6;
}

////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// 模块描述：RPY各个友元函数的实现。
// 作者：葛浏昊
// 日期：2011-3-18
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
void RPY::RPY2Trans(TRANS& t6)
{
	double pusi = orient.dx*PI/180;    //yaw    单位转换为弧度
	double theta = orient.dy*PI/180;   //pitch
	double fi = orient.dz*PI/180;      //roll

	t6.rot.mem[0][0] = cos(fi)*cos(theta);    //n
	t6.rot.mem[1][0] = sin(fi)*cos(theta);
	t6.rot.mem[2][0] = -sin(theta);

	t6.rot.mem[0][1] = cos(fi)*sin(theta)*sin(pusi)-sin(fi)*cos(pusi);   //o
	t6.rot.mem[1][1] = sin(fi)*sin(theta)*sin(pusi)+cos(fi)*cos(pusi);
	t6.rot.mem[2][1] = cos(theta)*sin(pusi);

	t6.rot.mem[0][2] = cos(fi)*sin(theta)*cos(pusi)+sin(fi)*sin(pusi);   //a
	t6.rot.mem[1][2] = sin(fi)*sin(theta)*cos(pusi)-cos(fi)*sin(pusi);
	t6.rot.mem[2][2] = cos(theta)*cos(pusi);

	t6.pos=pos;
}

void RPY::RPY2Quat( Quaternion& quat )
{
	TRANS t6;
	this->RPY2Trans(t6);
	t6.Trans2Quat(quat);
}

////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// 模块描述：Euler各个友元函数的实现。
// 作者：葛浏昊
// 日期：2011-3-18
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
void Euler::Euler2Trans(TRANS& t6)
{
	double fi = orient.dx*PI/180;    //单位转换为弧度
	double theta = orient.dy*PI/180;   
	double pusi = orient.dz*PI/180;      

	t6.rot.mem[0][0] = cos(fi)*cos(theta)*cos(pusi)-sin(fi)*sin(pusi);   //n
	t6.rot.mem[1][0] = sin(fi)*cos(theta)*cos(pusi)+cos(fi)*sin(pusi);
	t6.rot.mem[2][0] = -sin(theta)*cos(pusi);
	
	t6.rot.mem[0][1] = -cos(fi)*cos(theta)*sin(pusi)-sin(fi)*cos(pusi);  //o
	t6.rot.mem[1][1] = -sin(fi)*cos(theta)*sin(pusi)+cos(fi)*cos(pusi);
	t6.rot.mem[2][1] = sin(theta)*sin(pusi);
	
	t6.rot.mem[0][2] = cos(fi)*sin(theta);   //a
	t6.rot.mem[1][2] = sin(fi)*sin(theta);
	t6.rot.mem[2][2] = cos(theta);
	
	t6.pos = pos;
}

void Euler::Euler2Quat( Quaternion& quat )
{
	TRANS t6;
	this->Euler2Trans(t6);
	t6.Trans2Quat(quat);
}

////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// 模块描述：Quaternion各个友元函数的实现。
// 作者：葛浏昊
// 日期：2012-4-26
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
bool Quaternion::Quat2RPY( RPY& rpy )
{
	TRANS t6;
	this->Quat2TRANS(t6);
	t6.pos = rpy.pos;
	return t6.Trans2RPY(rpy);
}

void Quaternion::Quat2TRANS( TRANS& trans )
{
	trans.rot.mem[0][0] = 1-2*(y*y+z*z); trans.rot.mem[0][1] = 2*(x*y-w*z); trans.rot.mem[0][2] = 2*(x*z+w*y);
	trans.rot.mem[1][0] = 2*(x*y+w*z); trans.rot.mem[1][1] = 1-2*(x*x+z*z); trans.rot.mem[1][2] = 2*(y*z-w*x);
	trans.rot.mem[2][0] = 2*(x*z-w*y); trans.rot.mem[2][1] = 2*(y*z+w*x); trans.rot.mem[2][2] = 1-2*(x*x+y*y);
}

Quaternion& Quaternion::operator+=( const Quaternion& rhs )
{
	w += rhs.w;
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;

	return *this;
}

Quaternion Quaternion::Double( Quaternion p, Quaternion q )
{
	Quaternion res;
	double dot = p*q;
	res = (2*dot)*q - p;
//	res = 2*q - p;
	return res;
}

Quaternion Quaternion::Bisect( Quaternion p, Quaternion q )
{
	Quaternion res, temp = p+q;
	double mod = sqrt( temp*temp );
	res = temp/mod;
	return res;
}

Quaternion Quaternion::Conjugate() const
{
	Quaternion res;
	res.w = this->w;
	res.x = -this->x;
	res.y = -this->y;
	res.z = -this->z;
	return res;
}

Quaternion Quaternion::Inverse() const
{
	return this->Conjugate()/ (this->mod()*this->mod());
}

double Quaternion::mod() const
{
	return sqrt( this->w*this->w + this->x*this->x + this->y*this->y + this->z*this->z );
}

Quaternion operator+( const Quaternion& lhs,const Quaternion& rhs )
{
	Quaternion ret(lhs);
	ret += rhs;
	return ret;
}

Quaternion operator-( const Quaternion& lhs,const Quaternion& rhs )
{
	Quaternion ret;
	ret.w = lhs.w-rhs.w; ret.x = lhs.x-rhs.x; ret.y = lhs.y-rhs.y; ret.z = lhs.z-rhs.z;
	return ret;
}

Quaternion operator*( const double& lhs,const Quaternion& rhs )
{
	Quaternion ret(rhs);
	ret.w *= lhs;
	ret.x *= lhs;
	ret.y *= lhs;
	ret.z *= lhs;

	return ret;
}

Quaternion operator*( const Quaternion& lhs,const double& rhs )
{
	return rhs*lhs;
}

double operator*( const Quaternion& lhs,const Quaternion& rhs )
{
	return lhs.w*rhs.w + lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z;
}

Quaternion operator/( const Quaternion& lhs,const double& rhs )
{
	return (1/rhs)*lhs;
}

bool Quat_Interp( Quaternion& Q_result,const Quaternion& Q1,const Quaternion& Q2,double r )
{
	if (r<0 || r>1)
	{
		return false;
	}

	Quaternion q2 = Q2;
	double dot_product = Q1.w*Q2.w + Q1.x*Q2.x + Q1.y*Q2.y + Q1.z*Q2.z;
	double theta1 = acos( dot_product);
	double theta2 = acos( -dot_product);
	double theta = theta1;
	if ( theta1>theta2 )
	{
		theta = theta2;			// 取较小的角度，使插补路径最短
		q2 = (-1.0)*Q2;
	}
	
	if (isZero(theta))
	{
		Q_result = Q1;
	}
	else
	{
		// 四元数球面插补
//		Q_result = Quaternion( (sin((1-r)*theta) * Q1 + sin(r*theta) * q2) / sin(theta) );
	}

	return true;
}

bool Quat_Bez3rdInterp( Quaternion& Q_result, const Quaternion& Q1,const Quaternion& Q2,const Quaternion& a1,const Quaternion& b2,double u )	// 三阶贝塞尔曲线
{
	if (u<0 || u>1)
	{
		return false;
	}
	// 四元数球面Bezier样条插补
	Quaternion p00 = Q1, p10 = a1, p20 = b2, p30 = Q2;
	Quaternion p01, p11, p21, p02, p12;

	Quat_Interp(p01, p00, p10, u);
	Quat_Interp(p11, p10, p20, u);
	Quat_Interp(p21, p20, p30, u);

	Quat_Interp(p02, p01, p11, u);
	Quat_Interp(p12, p11, p21, u);

	Quat_Interp(Q_result, p02, p12, u);
	return true;
}

bool Quat_ArcInterp( Quaternion& Q_result, const Quaternion& Q1,const Quaternion& Q2,const Quaternion& Q3,double cur_arc,double arc1,double arc2 )
{
	Quaternion a1 = Q1;
	Quaternion a2 = Quaternion::Bisect( Quaternion::Double(Q1,Q2), Q3);
	Quaternion b2 = Quaternion::Double( a2, Q2);
	Quaternion b3 = Q3;

	if (cur_arc <= arc1)
	{
		return Quat_Bez3rdInterp(Q_result, Q1, Q2, a1, b2, cur_arc/arc1);
	}
	else if (cur_arc<=arc1 + arc2)
	{
		return Quat_Bez3rdInterp(Q_result, Q2, Q3, a2, b3, (cur_arc-arc1)/arc2);
	}
	return false;
}

bool Quat_Bez2ndInterp( Quaternion& Q_result, const Quaternion& Q1,const Quaternion& Q2,const Quaternion& Q3,double u )	// 二阶贝塞尔曲线
{
	if (u<0 || u>1)
	{
		return false;
	}
	// 四元数球面Bezier样条插补
	Quaternion p00 = Q1, p10 = Q2, p20 = Q3;
	Quaternion p01, p11, p02;
	
	Quat_Interp(p01, p00, p10, u);
	Quat_Interp(p11, p10, p20, u);
	
	Quat_Interp(Q_result, p01, p11, u);
	return true;
}

Quaternion operator^( const Quaternion& lhs,const Quaternion& rhs )		// 四元数的直乘
{
	Quaternion res;
	res.w = lhs.w*rhs.w - lhs.x*rhs.x - lhs.y*rhs.y - lhs.z*rhs.z;
	res.x = lhs.w*rhs.x + lhs.x*rhs.w + lhs.y*rhs.z - lhs.z*rhs.y;
	res.y = lhs.w*rhs.y - lhs.x*rhs.z + lhs.y*rhs.w + lhs.z*rhs.x;
	res.z = lhs.w*rhs.z + lhs.x*rhs.y - lhs.y*rhs.x + lhs.z*rhs.w;

	return res;
}

///////////////////////文献"A general construction scheme for unit quaternion curves with simple high order derivatives"中的方法///////////////////////////
Vector3D quat_log( const Quaternion& quat )
{
	Vector3D vec;
	double len = sqrt( pow(quat.x,2)+pow(quat.y,2)+pow(quat.z,2) );
	if ( fabs(len)<0.00001 )
	{
		return Vector3D(0,0,0);
	}
	double k = acos(quat.w)/len;
	vec.dx = k*quat.x;
	vec.dy = k*quat.y;
	vec.dz = k*quat.z;
	return vec;
}

Quaternion quat_exp( const Vector3D& vec )
{
	Quaternion quat;
	double len = sqrt( pow(vec.dx,2)+pow(vec.dy,2)+pow(vec.dz,2) );
	if ( fabs(len)<0.00001 )
	{
		return Quaternion(1,0,0,0);
	}
	quat.w = cos(len);
	double k = sin(len)/len;
	quat.x = k*vec.dx;
	quat.y = k*vec.dy;
	quat.z = k*vec.dz;
	return quat;
}

double BSplineBasicFun( int i, int k, const vector<double>& u, double t )	//第i段k次B样条基,Deboor递归算法
{
	if (i<1 || k<0 || u.size()<i+1)
	{
		return 0.0;
	}
	if (k==0)
	{
// 		if ( t<=u[i+1] && t>=u[i] )
// 		{
// 			return 1.0;
// 		}
		if ( t<u[i] && t>=u[i-1] )
		{
			return 1.0;
		}
		else
		{
			return 0.0;
		}
	}

// 	if (u.size()<i+k+1)
// 	{
// 		return 0.0;
// 	}
	double alpha=0.0, beta=0.0;
// 	if ( fabs(u[i+k-1]-u[i])<0.00001 )
// 	{
// 		alpha = 0.0;
// 	}
// 	else
// 	{
// 		alpha = (t-u[i]) / (u[i+k-1]-u[i]);
// 	}
// 	if ( fabs(u[i+k]-u[i+1])<0.00001 )
// 	{
// 		beta = 0.0;
// 	}
// 	else
// 	{
// 		beta = (u[i+k]-t) / (u[i+k]-u[i+1]);
// 	}
	if ( fabs(u[i+k-1]-u[i-1])==0 )//<0.00001 )
	{
		alpha = 0.0;
	}
	else
	{
		alpha = (t-u[i-1]) / (u[i+k-1]-u[i-1]);
	}
	if ( fabs(u[i+k]-u[i])==0 )//<0.00001 )
	{
		beta = 0.0;
	}
	else
	{
		beta = (u[i+k]-t) / (u[i+k]-u[i]);
	}
	return alpha*BSplineBasicFun(i, k-1, u, t) + beta*BSplineBasicFun(i+1, k-1, u, t);
}

double BSplineCumulBasicFun( int i, int k, const vector<double>& u, double t )	//第i段k次累积B样条基
{
//	i = i+1;
	if (i<0 || k<0 || u.size()<i+1)
	{
		return 0.0;
	}
	if ( t>u[i-1] && t<u[i+k-1] )
	{
		int j=0;
		double res = 0.0;
		for (j=i; j<=i+k+1; j++)
		{
			res += BSplineBasicFun(j+1,k-1,u,t);
		}
		return res;
	}
	else if ( t>u[i+k-2] )
	{
		return 1.0;
	}
	else
	{
		return 0.0;
	}
}

bool Quat_BSplineInterp( Quaternion& Q_result, const vector<Quaternion>& quat_list, double t, int num )	//B样条四元数插补，num表示第num段
{
	int k = 4;					// B样条曲线的阶次
	vector<double> u;
	int sz = quat_list.size();
	if (sz<=1 || t<num-1 || t>num)
	{
		return false;
	}
	int i=0;
	for (i=0; i<sz+50; i++)
	{
		u.push_back(double(i));
	}
	int l = num - k;
	if (l<0)
	{
		l = 0;
		Q_result = quat_exp( BSplineCumulBasicFun(0,k,u,t)*quat_log(quat_list[0]) );
	}
	else
	{
		if ( sz-1<l )
		{
			return false;
		}
		Q_result = quat_list[l];
	}
	for (i=l+1; i<=sz-1&&u[i]<t; i++)
	{
		Quaternion temp_quat = quat_list[i-1].Inverse() ^ quat_list[i];
		Q_result = Q_result ^ quat_exp( BSplineCumulBasicFun(i,k,u,t)*quat_log(temp_quat) );
	}
	return true;
}
