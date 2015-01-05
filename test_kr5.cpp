#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <string>
#include <cmath>
#include "robot.h"
#include "KR5ARC_robot.h"

using namespace de;

int program_jpos(vector<JAngle> &angle, vector<JAngle> &ex_angle, char *path);
int program_cpos(vector<RPY> &rpy, vector<JAngle> &ex_angle, char *path);

int main(int argc, char *argv[])
{
	std::vector<std::string> stl_path;
	if (argc != 2) {
		std::cout << "usage: ./kr5 kr5_line.pos" << std::endl;
		return 0;
	}
	robot_system<KR5ARC_robot> kr5(0, 60, 0.001, stl_path, argv[1]);
	kr5();
	
	return 0;
}
