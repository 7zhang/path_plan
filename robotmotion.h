// !@File
// ***************************************************************
// *<Pre>
// *文件名：robotmotion.h
// *功能描述：
// *作者：王成
// *日期：2011-3-7
// *</Pre>
// ***************************************************************
#ifndef __ROBOMOTION_H_
#define __ROBOMOTION_H_

#define IN
#define OUT
#define INOUT

//#include <assert.h>
#include "geometric.h"
#include "robotdata.h"

#define EXPENSIN 0.00001
const double Ts=0.010;  //步长



// !@class
// ******************************************************************
// *@<pre>
// *@类名：InterpolatePara
// *@作者：葛浏昊
// *@日期：2011-3-7
// *@</pre>
// ******************************************************************
class  InterpolatePara  
{
public:
	InterpolatePara();
	virtual ~InterpolatePara();

	double aimLen;
	double aimVel;
	double aimAcc;
	double aimJerk;			//期望的加加速度
	double beginVel;
	double endVel;
	double err[2];
	double timepoint[7];      //插补运动中的关键时间节点
	double lenpoint[5];       //插补运动中的关键路程节点

	bool cal_InterpolatePara();       //计算插补运动参数
	bool next_StepInterpolate(double &nextLen, double timecounter);  //按时间来计算插补步长

protected:
	double m_vel[2], m_acc[2], m_jerk[2];	//加、减速段的速度、加速度、加速度导数
};

// !@class
// ******************************************************************
// *@<pre>
// *@类名：IMotionTask	抽象类，不能声明对象
// *@作者：王成
// *@日期：2011-3-7
// *@</pre>
// ******************************************************************
class  IMotionTask
{
public:
	RouteSeg route;	
	InterpolatePara interPara;
	RoutePoint currPos;
	RoutePoint nextPos;
	
public:
//	bool init_route(double acc,double vel,RoutePoint BPos,RoutePoint EPos,RouteType type,double length=0);
	//纯虚函数
	virtual bool init_para() = 0;					//初始化路径参数
	virtual bool init_task(double velRate) = 0;		//初始化任务		
	virtual bool get_nextpoint(double velRate) = 0;	//获取下一插补点
	virtual bool Is_EndInter() = 0;					//判断插补是否结束

	virtual double flyby_checkVar(double radius) = 0;	//flyby中检查路径参数
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang) = 0;	//flyby中获取过渡段起始点及其单位切向量
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang) = 0;	//flyby中获取过渡段终止点及其单位切向量

	bool get_MotionTime(double& time){				//获取插补时间
			if(interPara.timepoint[6]==0)
			{
				time = 0;
				return false;
			}
			time = interPara.timepoint[6];
			return true;
	}
	virtual bool get_MotionRatio(double& ratio) = 0;//获取已插补的比例
};

// !@class
// ******************************************************************************
// *@<pre>
// *@类名：
// *@描述：Flyby运算接口
// *@作者：王成
// *@日期：2011-4-28
// *@</pre>
// ******************************************************************************
class  IFlybyAlg
{
	virtual double flyby_checkVar(double radius) = 0;	//flyby中检查路径参数
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang) = 0;	//flyby中获取过渡段起始点及其单位切向量
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang) = 0;	//flyby中获取过渡段终止点及其单位切向量	
};


// !@class
// ******************************************************************************
// *@<pre>
// *@类名：JointMTask
// *@描述：
// *@作者：王成
// *@日期：2011-4-28
// *@</pre>
// ******************************************************************************
class  JointMTask :public IMotionTask
{
public:
	JointMTask();
	bool init_para();					//初始化路径参数
	bool init_task(double velRate);		//初始化任务	
	bool get_nextpoint(double velRate);	//获取下一插补点
	bool Is_EndInter();					//判断插补是否结束

	double flyby_checkVar(double radius);	//flyby中检查路径参数
	bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang);	//flyby中获取过渡段起始点及其单位切向量
	bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang);	//flyby中获取过渡段终止点及其单位切向量

	void setScale(int * velScale, int * accScale, int *jerkScale);   //设定各关节的速度、加速度、加加速度比例
	bool get_MotionPara(double& len, double& vel, double& acc, int& number, double& jerk);  //获取运动参数
	bool get_MotionPara(double& len, double& vel, double& acc, int& number);
	bool get_MotionPara(double& len);

	virtual bool get_MotionRatio(double& ratio){	//获取已插补的比例
		if( fabs(m_selLen)<EXPENSIN )
		{
			return false;
		}
		ratio = currLen/m_selLen;
		if( ratio>1 || ratio<0 )
		{
			return false;
		}
		return true;
	}

protected:
	JAngle m_angBegin2End;	//起点到终点的关节角	
	double m_timecounter;	//记录插补时间
	int m_selNumber;		//选定的关节编号（0-5）
	double m_selLen;		//选定的路程差值
	double m_selVel;		//选定的运动速度
	double m_selAcc;		//选定的运动加速度
	double m_selJerk;		//选定的运动加加速度
	double m_velRate;       //最大运动速度的百分比
	double m_saveLen;
	int Velo_Max[6];		//单位:度/秒   {70,70,70,70,70,150}
	int Acce_Max[6];		//单位:度/秒方 {200,200,200,600,600,900} 
	int Jerk_Max[6];		//单位:度/秒立方 {200,200,200,600,600,900} 
	
public:
//	RoutePoint currPos;
//	RoutePoint nextPos;
	double currLen;
	double nextLen;
};

class  LineMTask :public IMotionTask
{
public:
	LineMTask();
	virtual bool init_para();					//初始化路径参数
	virtual bool init_task(double velRate);		//初始化任务	
	virtual bool get_nextpoint(double velRate);	//获取下一插补点
	virtual bool Is_EndInter();					//判断插补是否结束

	virtual double flyby_checkVar(double radius);	//flyby中检查路径参数
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang);	//flyby中获取过渡段起始点及其单位切向量
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang);	//flyby中获取过渡段终止点及其单位切向量

	bool get_MotionPara(double& len);
	bool get_MotionPara(double& len,double& vel, double& acc);

	virtual bool get_MotionRatio(double& ratio){	//获取已插补的比例
		if( fabs(m_len)<EXPENSIN )
		{
			return false;
		}
		ratio = currLen/m_len;
		if( ratio>1 || ratio<0 )
		{
			return false;
		}
		return true;
	}

protected:
	RPY m_rpyBegin2End;		//起点指向终点的RPY坐标
	double m_timecounter;	//记录插补时间
	double m_len;			//路程差值
	double m_vel;			//运动速度
	double m_velRate;       //最大运动速度的百分比
	double m_saveLen;

public:
//	RoutePoint currPos;
//	RoutePoint nextPos;
	double currLen;
	double nextLen;
};

class  ArcMTask :public IMotionTask
{
public:
	ArcMTask();
	virtual bool init_para();					//初始化路径参数
	virtual bool init_task(double velRate);		//初始化任务	
	virtual bool get_nextpoint(double velRate);	//获取下一插补点
	virtual bool Is_EndInter();					//判断插补是否结束

	virtual double flyby_checkVar(double radius);	//flyby中检查路径参数
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang);	//flyby中获取过渡段起始点及其单位切向量
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang);	//flyby中获取过渡段终止点及其单位切向量
	
	bool get_MotionPara(double& arc);
	bool get_MotionPara(double& arc,double& r,double& vel, double& acc);

	virtual bool get_MotionRatio(double& ratio){	//获取已插补的比例
		if( fabs(m_arc)<EXPENSIN )
		{
			return false;
		}
		ratio = currArc/m_arc;
		if( ratio>1 || ratio<0 )
		{
			return false;
		}
		return true;
	}

protected:
	RPY m_rpyBegin2Mid;		// 起点指向中点的RPY坐标
	RPY m_rpyMid2End;		// 中点指向末点的RPY坐标
	double m_radBegin2Mid;	// 起点到中点的弧度
	double m_radMid2End;	// 中点到末点的弧度
	double m_radBegin2End;	// 起点到末点的弧度
	double m_timecounter;	// 记录插补时间
	double m_r;				// 半径
	TRANS m_T;				// 圆弧坐标齐次变换矩阵,pos为圆心坐标
	double m_arc;			// 起点到末点的弧长
	double m_arc1;			// 起点到中点的弧长
	double m_arc2;			// 中点到末点的弧长
	double m_vel;			// 选定的运动速度
	double m_velRate;       // 最大运动速度的百分比
	double m_saveArc;

public:
	RoutePoint midPos;		// 中间示教点（用于圆弧插补中）
//	RoutePoint currPos;
//	RoutePoint nextPos;
	double currArc;
	double nextArc;
};

// !@class
// ******************************************************************************
// *@<pre>
// *@类名：CirMTask
// *@描述：完成整圆插补的功能（给定三个点，即插补出整圆，区别于ArcMTask）
// *@作者：葛浏昊
// *@日期：2012-02-03
// *@</pre>
// ******************************************************************************
class  CirMTask :public IMotionTask
{
public:
	CirMTask();
	virtual bool init_para();					//初始化路径参数
	virtual bool init_task(double velRate);		//初始化任务	
	virtual bool get_nextpoint(double velRate);	//获取下一插补点
	virtual bool Is_EndInter();					//判断插补是否结束
	
	virtual double flyby_checkVar(double radius);	//flyby中检查路径参数
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang);	//flyby中获取过渡段起始点及其单位切向量
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang);	//flyby中获取过渡段终止点及其单位切向量
	
	bool get_MotionPara(double& arc);
	bool get_MotionPara(double& arc,double& r,double& vel, double& acc);
	
	virtual bool get_MotionRatio(double& ratio){	//获取已插补的比例
		if( fabs(m_arc)<EXPENSIN )
		{
			return false;
		}
		ratio = currArc/(m_arc);
		if( ratio>1 || ratio<0 )
		{
			return false;
		}
		return true;
	}
	
protected:
	RPY m_rpyBegin2Mid;		// 起点指向中点的RPY坐标
	RPY m_rpyMid2End;		// 中点指向末点的RPY坐标
	double m_radBegin2Mid;	// 起点到中点的弧度
	double m_radMid2End;	// 中点到末点的弧度
	double m_radBegin2End;	// 起点到末点的弧度
	double m_timecounter;	// 记录插补时间
	double m_r;				// 半径
	TRANS m_T;				// 圆弧坐标齐次变换矩阵,pos为圆心坐标
	double m_arc;			// 起点到末点的弧长
	double m_arc1;			// 起点到中点的弧长
	double m_arc2;			// 中点到末点的弧长
	double m_vel;			// 选定的运动速度
	double m_velRate;       // 最大运动速度的百分比
	double m_saveArc;
	
public:
	RoutePoint midPos;		// 中间示教点（用于圆弧插补中）
	double currArc;
	double nextArc;
};

class  FlybyMTask :public IMotionTask
{
public:
	FlybyMTask();
	virtual bool init_para();						//初始化路径参数
	virtual bool init_task(double velRate);			//初始化任务	
	virtual bool get_nextpoint(double velRate);		//获取下一插补点
	virtual bool Is_EndInter();						//判断插补是否结束

	virtual double flyby_checkVar(double radius);	//flyby中检查路径参数
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang);	//flyby中获取过渡段起始点及其单位切向量
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang);	//flyby中获取过渡段终止点及其单位切向量

	bool get_MotionPara(double& len);
	Vector3D get_curvepoint(Matrix4D MCurve,double t);		//获取弗格森曲线上的点
	double get_curvelength(const Matrix4D& MCurve);			//获取弗格森曲线的长度
	Matrix4D get_curveMatrix(const Vector3D& beginPos,const Vector3D& endPos,
							const Vector3D& beginTang,const Vector3D& endTang);		//获取flyby插补所用的弗格森曲线矩阵
	double get_minDist(Matrix4D MCurve,Vector3D point);		//获取点到弗格森曲线的最短距离

	virtual bool get_MotionRatio(double& ratio){	//获取已插补的比例
		if( fabs(m_len)<EXPENSIN )
		{
			return false;
		}
		ratio = currLen/m_len;
		if( ratio>1 || ratio<0 )
		{
			return false;
		}
		return true;
	}

protected:
	RPY m_rpyBegin2End;		// 起点指向终点的RPY坐标
	double m_timecounter;	// 记录插补时间
	double m_len;			// 曲线长度
	double m_vel;			// 选定的运动速度
	double m_velRate;       // 最大运动速度的百分比
	double m_saveLen;

public:
	Matrix4D m_MCurve;		// 弗格森曲线矩阵，包含起始、终止点信息p0,p1和起始、终止向量T0,T1（输入量，已规划好）
//	RoutePoint currPos;
//	RoutePoint nextPos;
	double currLen;
	double nextLen;
	double path_radius1;	//过渡路径的起始点到所逼近示教点的距离
	double path_radius2;	//过渡路径的终止点到所逼近示教点的距离
	double path_dist;		//弗格森曲线到所逼近示教点的所允许的最短距离
};

class  SplineMTask :public IMotionTask
{
public:
	SplineMTask();
	virtual bool init_para();					//初始化路径参数
	virtual bool init_task(double velRate);		//初始化任务	
	virtual bool get_nextpoint(double velRate);	//获取下一插补点
	virtual bool Is_EndInter();					//判断插补是否结束
	
	virtual double flyby_checkVar(double radius);	//flyby中检查路径参数
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang);	//flyby中获取过渡段起始点及其单位切向量
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang);	//flyby中获取过渡段终止点及其单位切向量
	
	virtual bool get_MotionRatio(double& ratio){	//获取已插补的比例
		if( fabs(m_len)<EXPENSIN )
		{
			return false;
		}
		ratio = currLen/(m_len);
		if( ratio>1 || ratio<0 )
		{
			return false;
		}
		return true;
	}

protected:
	RPY m_rpyBegin2End;		// 起点指向终点的RPY坐标
	double m_timecounter;	// 记录插补时间
	double m_vel;			// 选定的运动速度
	double m_velRate;       // 最大运动速度的百分比
	double m_saveLen;
	
public:
	//	RoutePoint currPos;
	//	RoutePoint nextPos;
	double m_len;			// 路程差值
	double m_tense;			// 张力系数
	double currLen;			// 输出量
	double nextLen;			// 输出量
	Vector3D ddBeginPos;	// 起点曲线的二阶导数
	Vector3D ddEndPos;		// 终点曲线的二阶导数
	
	// add by geliuhao 20120713
//	vector<Quaternion> m_QuatList;		// 存储样条曲线上的所有姿态点
	unsigned int m_num;			// 记录该段曲线在样条曲线中的序号,从1开始
	Quaternion m_AuxQuat;		// 用于生成贝塞尔曲线的辅助点
};

typedef CTypedPtrList	<CPtrList,SplineMTask*>	SplineMTaskList;	//样条曲线任务链表
typedef CTypedPtrList	<CPtrList,Vector3D*>	Vector3DList;		//三维向量链表

// [[ <FUNCTION GROUP>
	//计算插补时速度变化
	  BOOL VeloStep(double Vcurrent, double Vaim, double accel, double& Vnext);  
	 
	 //给出空间三点求圆心、半径和转换矩阵
	  BOOL GetCenRT(Point3D& p1, Point3D& p2, Point3D& p3, Point3D& cen, double& r, TRANS& T);	
	
	 //求圆上顺序三点之间的圆心角
	  void GetCenAngle(const Point3D& q1,const Point3D& q2,const Point3D& q3,double& rad_12,double& rad_23,double& rad_13);	
	 
	  bool add_flyby(INOUT IMotionTask *seg1, INOUT IMotionTask *seg2, OUT FlybyMTask *seg3);
	  bool _IsEndInter(double selLen,double nextLen );	//判断插补路程是否已完成

	  bool get_SplinePara(SplineMTaskList* splineList);		//求解张力样条曲线参数	 
	  bool solve_SplinePara(Vector3DList* splinePointList, double tense, Vector3DList* ddPointList);	//解样条曲线参数
	  bool get_SplinePointSlope(Vector3D p1, Vector3D p2, Vector3D p3, double x1, double x2, double x3, Vector3D& slope);	//获得样条曲线端点的斜率(按抛物线拟合)
	  void free_Vector3DList(Vector3DList* plist);			//释放链表内存空间
	  void free_SplineMTaskList(SplineMTaskList* plist);		//释放链表内存空间

	  bool get_SplineQuatPara(SplineMTaskList* splineList);		// 获取样条插补中的四元数姿态参数
// ]] </FUNCTION GROUP>

#endif /*__ROBOMOTION_H__*/
