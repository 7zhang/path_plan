#include "system_state.h"

int system_state::m_axis_nr;
int system_state::m_auxiliary_variable_nr;
//	int m_sub_cri_nr;

Vector3D system_state::m_p;
Vector3D system_state::m_n;
Vector3D system_state::m_t;

std::vector<axis> system_state::m_axes;
std::vector<axis> system_state::m_auxiliary_variable;
std::vector<int> system_state::m_map;

std::vector<teach_point> system_state::m_teach_points;
std::vector<double> system_state::m_weight;
