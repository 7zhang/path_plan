#include "Kunshan_robot.h"

static void kunshan_robot::init()
{
	m_axis_nr = 9;
	m_axes.resize(9);
	m_axes[0] = axis(-150.0, 180.0, 50.0, 10.0, 10, 0);
	m_axes[1] = axis(-125.0, 30.0, 50.0, 10.0, 10, 0);
	m_axes[2] = axis(-120.0, 150.0, 50.0, 10.0, 10, 0);
	m_axes[3] = axis(-180.0, 180.0, 50.0, 10.0, 10, 0);
	m_axes[4] = axis(-120.0, 120.0, 50.0, 10.0, 10, 0);
	m_axes[5] = axis(-180.0, 180.0, 50.0, 10.0, 10, 0);
	m_axes[6] = axis(0.0, 90.0, 50.0, 10.0, 10, 0);
	m_axes[7] = axis(-185.0, 185.0, 50.0, 10.0, 10, 0);
	m_axes[8] = axis(-1700.0, 1600.0, 50.0, 10.0, 10, 0);

	m_auxiliary_variable_nr = 4;
	m_auxiliary_variable.resize(4);
	m_auxiliary_variable[0] = axis(0.0, 15.0, 3.0, 3.0, 10, 0); 
	m_auxiliary_variable[1] = axis(0.0, 15.0, 3.0, 3.0, 10, 0); 
	m_auxiliary_variable[2] = axis(75.0, 105.0, 3.0, 3.0, 10, 0); 
	m_auxiliary_variable[3] = axis(0.0, 1.0, 3.0, 3.0, 10, 3);
}
