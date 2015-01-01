# Program('path_plan', ['axis.cpp', 'exp_criteria.cpp', 'function.cpp', 'geometric.cpp', 'KunShanJacobi.cpp',
# 		     'KunShan_robot.cpp', 'load_seam.cpp', 'robotdata.cpp', 'robotkinematic.cpp', 'sine_criteria.cpp',
# 		     'system_state.cpp', 'teach_point.cpp', 'calc_criteria.cpp', 'test.cpp', 'Transform.cpp'],
# 		     LIBS = ['boost_program_options', 'pthread', 'boost_system', 'boost_thread'],
# 		     CCFLAGS = '-std=c++11 -O2')
Program('server', ['plan_server.cpp', 'geometric.cpp', 'system_state.cpp', 'axis.cpp', 'exp_criteria.cpp', 'function.cpp', 'KunShanJacobi.cpp', 'KunShan_robot.cpp', 'load_seam.cpp', 'robotdata.cpp', 'robotkinematic.cpp', 'sine_criteria.cpp','teach_point.cpp', 'calc_criteria.cpp','Transform.cpp'], LIBS = ['jsoncpp', 'jsonrpccpp-common', 'jsonrpccpp-server', 'microhttpd', 'boost_program_options', 'pthread', 'boost_system', 'boost_thread'], CCFLAGS = '-g -std=c++11')
Program('client', ['client.cpp', 'load_seam.cpp', 'geometric.cpp'], LIBS = ['jsoncpp', 'jsonrpccpp-common', 'jsonrpccpp-client', 'boost_program_options', 'pthread', 'boost_system', 'boost_thread'], 
		  CCFLAGS = '-g -std=c++11')