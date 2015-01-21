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
#include "plan_strategy.h"
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
//	job myjob(p, n, t);
//	robot_system<kunshan_robot> kunshan("kunshan robot", 6, 60, 0.001, stl_path, myjob);

	plan_strategy<T> *work = new plan_strategy<T>(p, n, t);

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

//	std::cout << work->get_sys_info();
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

	if (! request.isMember("para2"))
	{
		response = -1;
		return;
	}
	Json::Value index = request["para2"];
	int k = index.asInt();

	std::vector<double> m_axes_values;
	std::vector<double> m_auxiliary_variable_values;
	std::vector<double> m_sub_cri_axis;
	std::vector<double> m_sub_cri_aux;
	std::vector<double> m_sub_cri_teach;
	double m_cri;
//	std::pair<int, int> ret;
	int status = m_works[job_id]->get_status();

	if (status < 4) {
		response["size"] = 0;
		response["finished"] = 0;
		return;
	}
	std::pair<int, int> ret = m_works[job_id]->result->get_finish_rate(k, m_axes_values, 
								   m_auxiliary_variable_values,
								   m_sub_cri_axis,
								   m_sub_cri_aux,
								   m_sub_cri_teach,
								   m_cri);

	response["size"] = ret.first;
	response["finished"] = ret.second;

	Json::Value axes_values;
	for (int i = 0; i < m_axes_values.size(); i++) {
		axes_values.append(m_axes_values[i]);
	}
	response["axes"] = axes_values;
	
	Json::Value auxiliary_variable_values;
	for (int i = 0; i < m_auxiliary_variable_values.size(); i++) {
		auxiliary_variable_values.append(m_auxiliary_variable_values[i]);
	}
	response["auxiliary"] = auxiliary_variable_values;

	Json::Value sub_cri_axis;
	for (int i = 0; i < m_sub_cri_axis.size(); i++) {
		sub_cri_axis.append(m_sub_cri_axis[i]);
	}
	response["cri_axis"] = sub_cri_axis;

	Json::Value sub_cri_aux;
	for (int i = 0; i < m_sub_cri_aux.size(); i++) {
		sub_cri_aux.append(m_sub_cri_aux[i]);
	}
	response["cri_aux"] = sub_cri_aux;

	Json::Value sub_cri_teach;
	for (int i = 0; i < m_sub_cri_teach.size(); i++) {
		sub_cri_teach.append(m_sub_cri_teach[i]);
	}
	response["cri_teach"] = sub_cri_teach;

	Json::Value cri;
	cri = m_cri;
	response["cri"] = cri;

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
//		ret = m_works[job_id]->set_sys_parameter(para_name, &ivalue, restart);	
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
//		ret = m_works[job_id]->set_sys_parameter(para_name, &dvalue, restart);	
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

int main(int argc, const char *argv[])
{
	if (argc < 2) {
		printf("usage server 8383\n");
		return -1;
	}
	jsonrpc::HttpServer httpserver(atoi(argv[1]));
	KR5ARC_robot::cd_initialize();
	path_plan_server<KR5ARC_robot > s(httpserver);
	s.StartListening();
	std::cout << "server started" << std::endl;
	getc(stdin);
	s.StopListening();
	return 0;
}

