#ifndef _JOB_H_
#define _JOB_H_

#include <string>
#include <iostream>
#include "load_seam.h"

class job
{
public:
	int m_size;
	std::vector<Vector3D> m_normal, m_tangent, m_point;
	int m_pos;
	std::vector<double> m_para;
public:
	job(std::string seam) {
		if (!seam.empty()) {
			cerr << seam.c_str();
			std::cerr << "load seam file: " << seam << std::endl;
			if (load_seam(seam.c_str(), m_point, m_normal, m_tangent)) {
				std::cerr << "load_seam error: " << seam << std::endl;
			}

			for (int i = 0; i < m_normal.size(); i++) {
				m_normal[i].unitize();
			}

			for (int i = 0; i < m_tangent.size(); i++) {
				m_tangent[i].unitize();
			}
			m_size = m_normal.size();
		} else {
			m_size = 0;
		}
	}

        job(int pos, std::vector<double> para, std::vector<Vector3D>& p, std::vector<Vector3D>& n, std::vector<Vector3D>& t) :
	m_pos(pos), m_para(para), m_point(p), m_normal(n), m_tangent(t) {
			for (int i = 0; i < m_normal.size(); i++) {
				m_normal[i].unitize();
			}

			for (int i = 0; i < m_tangent.size(); i++) {
				m_tangent[i].unitize();
			}
		m_size = p.size();
	}
	
	const Vector3D& get_n(int index) { return m_normal[index]; }
	const Vector3D& get_t(int index) { return m_tangent[index]; }
	const Vector3D& get_p(int index) { return m_point[index]; }

	std::vector<Vector3D>& get_n() { return m_normal; }
	std::vector<Vector3D>& get_t() { return m_tangent; }
	std::vector<Vector3D>& get_p() { return m_point; }

	int get_size() { return m_size; }
};


#endif /* _JOB_H_ */
