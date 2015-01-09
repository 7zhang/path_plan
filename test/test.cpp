#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <string>
#include <cmath>
#include "robot.h"
#include "kunshan/kunshan_robot.h"

using namespace de;

int program_jpos(vector<JAngle> &angle, vector<JAngle> &ex_angle, char *path);
int program_cpos(vector<RPY> &rpy, vector<JAngle> &ex_angle, char *path);

int main(int argc, char *argv[])
{
	std::vector<std::string> stl_path;
	robot_system<kunshan_robot> kunshan(0, 60, 0.001, stl_path, "test1.pos");
	kunshan();
	
	return 0;
}
