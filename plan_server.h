#ifndef _PLAN_SERVER_H_
#define _PLAN_SERVER_H_

#include <jsonrpccpp/server.h>
#include <map>
//class AbstractStubServer : public jsonrpc::AbstractServer<AbstractStubServer>
template <typename T>
class path_plan_server: public jsonrpc::AbstractServer<path_plan_server<T> >
{
public:
	std::vector<T*> m_works; 
public:
	path_plan_server(jsonrpc::AbstractServerConnector &conn, jsonrpc::serverVersion_t type = jsonrpc::JSONRPC_SERVER_V2) : jsonrpc::AbstractServer<path_plan_server>(conn, type) {
		this->bindAndAddMethod(jsonrpc::Procedure("jsontest", jsonrpc::PARAMS_BY_NAME, jsonrpc::JSON_INTEGER, "para1", jsonrpc::JSON_ARRAY, "para2", jsonrpc::JSON_ARRAY, "para3", jsonrpc::JSON_ARRAY, NULL), &path_plan_server::jsontest);
	}

	void jsontest(const Json::Value &request, Json::Value &response);
};

#endif /* _PLAN_SERVER_H_ */
