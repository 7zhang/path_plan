#include "weld_point.h"

TRANS weld_point::get_trans()
{
	Vector3D m_y = m_n * m_t;
	return TRANS(m_t.dx, m_t.dy, m_t.dz,
		m_y.dx, m_y.dy, m_y.dz,
		m_n.dx, m_n.dy, m_n.dz,
		m_p.dx, m_p.dy, m_p.dz
		);
}

bool weld_point::operator < (weld_point r_wedpoint)
{
	return m_fi < r_wedpoint.m_fi;
}