#include <stdio.h>
#include <iostream>
#include <cmath>
#include <jsonrpccpp/server/connectors/httpserver.h>
#include <boost/thread.hpp>
#include "plan_server.h"
#include "geometric.h"
#include "robot.h"
#include "KunShan_robot.h"
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
	m_works.push_back(work);

	std::cout << work->get_sys_info();
	std::cout << "add job " << m_works.size() - 1 << std::endl << std::endl;

	boost::thread* th( new boost::thread(boost::ref(*work)) );
	boost::thread_group m_threads;
	m_threads.add_thread( th );

	response = m_works.size() - 1;
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
	if (job_id >= m_works.size()) {
		response = -1;
		return;
	}
	std::pair<int, int> ret = m_works[job_id]->get_finish_rate();
	response["size"] = ret.first;
	response["finished"] = ret.second;
}

int main()
{
	jsonrpc::HttpServer httpserver(8383);
	path_plan_server<robot_system<kunshan_robot> > s(httpserver);
	s.StartListening();
	getc(stdin);
	s.StopListening();
	return 0;
}

