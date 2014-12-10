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
			std::cout << "load_seam error: " << seam << std::endl;
		}

		m_size = m_normal.size();
	};
	
	const Vector3D& get_n(int index);
	const Vector3D& get_t(int index);
	const Vector3D& get_p(int index);

	int get_size() { return m_size; }

	virtual ~job();
};


#endif /* _JOB_H_ */
