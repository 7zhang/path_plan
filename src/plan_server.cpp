#include <stdio.h>
#include <iostream>
#include <cmath>
#include <jsonrpccpp/server/connectors/httpserver.h>
#include <boost/thread.hpp>
#include "plan_server.h"
#include "geometric.h"
#include "robot.h"
//#include "kunshan/kunshan_robot.h"
#include "kuka/kr5arc_robot.h"
using namespace std;
using namespace jsonrpc;

template <typename T>
void path_plan_server<T>::start_new(const Json::Value &request, Json::Value &response)
{
//	cout << request.toStyledString() << endl;
	if (! request.isObject()) {
		response = -1;
		return;
	}

	if (!(request.isMember("para1") && request.isMember("para2") && request.isMember("para3")))
	{
		response = -1;
		return;
	}
	std::vector<Vector3D> p, n, t;
	for (int i = 0; i < request["para1"].size(); i++) {
		if (request["para1"].isArray() && request["para2"].isArray() && request["para3"].isArray()
		    && request["para1"][i].isArray() && request["para2"][i].isArray() && request["para3"][i].isArray()) {
			Vector3D tmp1(request["para1"][i][0].asDouble(), request["para1"][i][1].asDouble(), request["para1"][i][2].asDouble());
			p.push_back(tmp1);
			
			Vector3D tmp2(request["para2"][i][0].asDouble(), request["para2"][i][1].asDouble(), request["para2"][i][2].asDouble());
			n.push_back(tmp2);

			Vector3D tmp3(request["para3"][i][0].asDouble(), request["para3"][i][1].asDouble(), request["para3"][i][2].asDouble());
			t.push_back(tmp3);
		} else {
			response = -1;
			return;
		}
	}

	std::vector<std::string> stl_path;
	job myjob(p, n, t);
//	robot_system<kunshan_robot> kunshan("kunshan robot", 6, 60, 0.001, stl_path, myjob);
	T *work = new T(m_works.size(), 60, 0.001, stl_path, myjob);

	int job_id = 0;
	int i = 0;
	for (; i < m_works.size(); i++) {
		if (m_works[i] == NULL) {
			break;
		}
	}

	if (i == m_works.size()) {
		m_works.push_back(work);
		job_id = m_works.size() - 1;
	} else {
		m_works[i] = work;
		job_id = i;
	}

	std::cout << work->get_sys_info();
	std::cout << "add job " << m_works.size() - 1 << std::endl << std::endl;

	boost::thread* th( new boost::thread(boost::ref(*work)) );
	boost::thread_group m_threads;
	m_threads.add_thread( th );

	response = job_id;
}

template <typename T>
void path_plan_server<T>::get_finish_rate(const Json::Value &request, Json::Value &response)
{
//	cerr << request.toStyledString() << endl;
	if (! request.isObject()) {
		response = -1;
		return;
	}

	if (! request.isMember("para1"))
	{
		response = -1;
		return;
	}
	Json::Value j = request["para1"];

	int job_id = j.asInt();
	if (job_id < 0 || job_id >= m_works.size()) {
		response = -1;
		return;
	}
	std::pair<int, int> ret = m_works[job_id]->get_finish_rate();
	response["size"] = ret.first;
	response["finished"] = ret.second;
	if (ret.first < 0) {
		if (ret.second != -ret.first) {
			std::cerr << "job " << job_id << " stopped" << std::endl;
		} else {
			std::cerr << "job " << job_id << " finished" << std::endl;
		}
	} 
}

template <typename T>
void path_plan_server<T>::set_sys_parameter_int(const Json::Value &request, Json::Value &response)
{
	cout << "set_sys_parameter_int called" << endl;
	cout << request.toStyledString() << endl;
	if (! request.isObject()) {
		response = -1;
		return;
	}

	if (!(request.isMember("para1") && request.isMember("para2") && request.isMember("para3") && request.isMember("para4")))
	{
		response = -1;
		return;
	}

	std::string para_name = request["para1"].asString();
	Json::Value pvalue = request["para2"];
	int restart = request["para3"].asInt();
	int job_id = request["para4"].asInt();
	if (job_id < 0 ||  job_id >= m_works.size()) {
		response = -1;
		return;
	}

	int ivalue = 0;
	
	int ret = 0;
	if (para_name == "de_pop_size" || para_name == "de_thread_nr") {
		ivalue = pvalue.asInt();
		ret = m_works[job_id]->set_sys_parameter(para_name, &ivalue, restart);	
	} else {
		response = -1;
		return;
	}

	response = ret;
	std::cout << "job " << job_id << " parameter changed, ";
	if (restart) {
		std::cout << "job restarted" << std::endl;
	} else {
		std::cout << "job stopped" << std::endl;
	}
}

template <typename T>
void path_plan_server<T>::set_sys_parameter_double(const Json::Value &request, Json::Value &response)
{
	cout << "set_sys_parameter_double called" << endl;
	cout << request.toStyledString() << endl;
	if (! request.isObject()) {
		response = -1;
		return;
	}

	if (!(request.isMember("para1") && request.isMember("para2") && request.isMember("para3") && request.isMember("para4")))
	{
		response = -1;
		return;
	}

	std::string para_name = request["para1"].asString();
	Json::Value pvalue = request["para2"];
	int restart = request["para3"].asInt();
	int job_id = request["para4"].asInt();

	if (job_id < 0 ||  job_id >= m_works.size()) {
		response = -1;
		return;
	}
	double dvalue = 0.0;
	
	int ret = 0;
	if (para_name == "de_weight" || para_name == "de_crossover") {
		dvalue = pvalue.asDouble();
		ret = m_works[job_id]->set_sys_parameter(para_name, &dvalue, restart);	
	} else {
		response = -1;
		return;
	}

	response = ret;
	std::cout << "job " << job_id << " parameter changed, ";
	if (restart) {
		std::cout << "job restarted" << std::endl;
	} else {
		std::cout << "job stopped" << std::endl;
	}
}

int main()
{
	jsonrpc::HttpServer httpserver(8383);
	path_plan_server<robot_system<KR5ARC_robot> > s(httpserver);
	s.StartListening();
	getc(stdin);
	s.StopListening();
	return 0;
}

