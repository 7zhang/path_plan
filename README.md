path_plan
=========
this is the program planning redundancy robot path, for academic only

run make
flags = -O2 -lboost_program_options -lpthread -lboost_system -lboost_thread -std=c++11
all:
	g++ $(flags) robot_de_optimize.cpp function.cpp calc_criteria.cpp geometric.cpp \
	calc_state.cpp KunShanJacobi.cpp positioner.cpp robotkinematic.cpp robotdata.cpp \
	Transform.cpp load_seam.cpp
