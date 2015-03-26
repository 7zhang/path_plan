#ifndef _PLAN_SERVER_H_
#define _PLAN_SERVER_H_

#include <jsonrpccpp/server.h>
#include <map>
#include "robot.h"
#include "plan_strategy.h"
//class AbstractStubServer : public jsonrpc::AbstractServer<AbstractStubServer>
template <typename T>
class path_plan_server: public jsonrpc::AbstractServer<path_plan_server<T> >
{
public:
	std::vector<plan_strategy<T> *> m_works; 
public:
	path_plan_server(jsonrpc::AbstractServerConnector &conn, jsonrpc::serverVersion_t type = jsonrpc::JSONRPC_SERVER_V2) : jsonrpc::AbstractServer<path_plan_server>(conn, type) {
		this->bindAndAddMethod(jsonrpc::Procedure("start_new", jsonrpc::PARAMS_BY_NAME, jsonrpc::JSON_INTEGER, "para1", jsonrpc::JSON_ARRAY, "para2", jsonrpc::JSON_ARRAY, "para3", jsonrpc::JSON_ARRAY, "para4", jsonrpc::JSON_INTEGER, "para5", jsonrpc::JSON_ARRAY, NULL), &path_plan_server::start_new);
		
		this->bindAndAddMethod(jsonrpc::Procedure("get_finish_rate", jsonrpc::PARAMS_BY_NAME, jsonrpc::JSON_OBJECT,  "para1", jsonrpc::JSON_INTEGER, "para2", jsonrpc::JSON_INTEGER, NULL), &path_plan_server::get_finish_rate);

		this->bindAndAddMethod(jsonrpc::Procedure("set_sys_parameter_int", jsonrpc::PARAMS_BY_NAME, jsonrpc::JSON_INTEGER, "para1", jsonrpc::JSON_STRING, "para2", jsonrpc::JSON_INTEGER, "para3", jsonrpc::JSON_INTEGER, "para4", jsonrpc::JSON_INTEGER, NULL), &path_plan_server::set_sys_parameter_int);

		this->bindAndAddMethod(jsonrpc::Procedure("set_sys_parameter_double", jsonrpc::PARAMS_BY_NAME, jsonrpc::JSON_INTEGER, "para1", jsonrpc::JSON_STRING, "para2", jsonrpc::JSON_REAL, "para3", jsonrpc::JSON_INTEGER, "para4", jsonrpc::JSON_INTEGER, NULL), &path_plan_server::set_sys_parameter_double);
		/* this->bindAndAddMethod(jsonrpc::Procedure("insert_teach_point", jsonrpc::PARAMS_BY_NAME, jsonrpc::JSON_INTEGER, "para1", jsonrpc::JSON_STRING, "para2", jsonrpc::JSON_REAL, "para3", jsonrpc::JSON_INTEGER, "para4", jsonrpc::JSON_INTEGER, NULL), &path_plan_server::); */
	}

	void start_new(const Json::Value &request, Json::Value &response);
	void get_finish_rate(const Json::Value &request, Json::Value &response);
	void set_sys_parameter_int(const Json::Value &request, Json::Value &response);
	void set_sys_parameter_double(const Json::Value &request, Json::Value &response);
};

#endif /* _PLAN_SERVER_H_ */
