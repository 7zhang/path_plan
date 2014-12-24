#ifndef _JOB_H_
#define _JOB_H_

class job
{
private:
	int m_size;
	std::vector<Vector3D> m_normal, m_tangent, m_point;
public:
	job(std::string seam) {
		if (load_seam(seam.c_str(), m_point, m_normal, m_tangent)) {
			std::cerr << "load_seam error: " << seam << std::endl;
		}

		m_size = m_normal.size();

		std::cerr << "load seam file: " << seam << std::endl;
	};
	
	const Vector3D& get_n(int index) { return m_normal[index]; };
	const Vector3D& get_t(int index) { return m_tangent[index]; };
	const Vector3D& get_p(int index) { return m_point[index]; };

	int get_size() { return m_size; }
};


#endif /* _JOB_H_ */
