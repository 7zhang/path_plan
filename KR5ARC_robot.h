#ifndef _KR5ARC_ROBOT_H_
#define _KR5ARC_ROBOT_H_

#include "system_state.h"
#include "KR5ARC.h"

class KR5ARC_robot: public system_state
{
	IKinematicAlg *rob;

public:
KR5ARC_robot(int axis_nr, int auxiliary_variable_nr, 
	      const Vector3D& p, const Vector3D& n, 
	      const Vector3D& t, const std::vector<axis>& axes, 
	      const std::vector<axis>& auxiliary_variable, const std::vector<int>& map,
	      const std::vector<teach_point>& teach_points, const std::vector<double>& weight)
	: system_state(axis_nr,auxiliary_variable_nr, p, n, t,
		       axes, auxiliary_variable, map, teach_points, weight){ rob = new KR5ARC_RKA(); }
//	static void init();
	void check();
	void print_trans(std::string name, TRANS& trans);
	double operator() (de::DVectorPtr args);

	/* static void init(int& m_axis_nr, int& m_auxiliary_variable_nr, */
	/* 	  std::vector<axis>& m_axes, std::vector<axis>& m_auxiliary_variable, */
	/* 	  std::vector<int>& m_map, std::vector<teach_point>& m_teach_points, */
	/* 	  std::vector<double>& m_weight); */

	static void init(std::string& m_sys_name, 
					int& m_redundancy,
					int& m_axis_nr,
					int& m_auxiliary_variable_nr,
					std::vector<axis>& m_axes,
					std::vector<axis>& m_auxiliary_variable,
					std::vector<int>& m_map,
					std::vector<teach_point>& m_teach_points,
					std::vector<double>& m_weight);
	/**
	/*transformation about robot 
	/*@2014.1.4
	 */
	virtual TRANS RotateX(double angle);
	virtual TRANS RotateY(double angle);
	virtual TRANS RotateZ(double angle);
	virtual TRANS Trans(double x, double y, double z);

//	virtual TRANS get_gun_in_seam();
	virtual TRANS get_gun_in_seam(const JAngle& weld_angle);
	virtual TRANS getTransWorldToWorkpiece(JAngle ex_angle);
	virtual TRANS getTrans6ToTorch();
	virtual TRANS getTransWorldToBase(JAngle ex_angle);
	virtual bool InverseRobot(JAngle& Jointangle,const JAngle& lastJointangle,const TRANS& t6);
	virtual double get_jacobi_deter(JAngle& angle);	
};

#endif /* _KR5ARC_ROBOT_H_ */
