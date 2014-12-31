# Program('path_plan', ['axis.cpp', 'exp_criteria.cpp', 'function.cpp', 'geometric.cpp', 'KunShanJacobi.cpp',
# 		     'KunShan_robot.cpp', 'load_seam.cpp', 'robotdata.cpp', 'robotkinematic.cpp', 'sine_criteria.cpp',
# 		     'system_state.cpp', 'teach_point.cpp', 'calc_criteria.cpp', 'test.cpp', 'Transform.cpp'],
# 		     LIBS = ['boost_program_options', 'pthread', 'boost_system', 'boost_thread'],
# 		     CCFLAGS = '-std=c++11 -O2')
Program('server', 'plan_server.cpp', LIBS = ['jsoncpp', 'jsonrpccpp-common',
		   'jsonrpccpp-server', 'microhttpd'], CCFLAGS = '-g')
Program('client', ['client.cpp', 'load_seam.cpp', 'geometric.cpp'], LIBS = ['jsoncpp', 'jsonrpccpp-common', 'jsonrpccpp-client'], 
		  CCFLAGS = '-g')