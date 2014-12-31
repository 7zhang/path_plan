#ifndef _CLIENT_H_
#define _CLIENT_H_

#include <vector>
#include <string>
#include <jsonrpccpp/client.h>
#include <iostream>
#include "geometric.h"

using namespace std;

class path_plan_client : public jsonrpc::Client
{
    public:
        path_plan_client(jsonrpc::IClientConnector &conn, jsonrpc::clientVersion_t type = jsonrpc::JSONRPC_CLIENT_V2) : jsonrpc::Client(conn, type) {}

        int jsontest(std::vector<std::vector<Vector3D> >& para) throw (jsonrpc::JsonRpcException)
        {
		Json::Value p, n, t;
		Json::Value tmp;

		for (int i = 0; i < para[0].size(); i++) {
			tmp[0] = para[0][i].dx;
			tmp[1] = para[0][i].dy;
			tmp[2] = para[0][i].dz;
			p.append(tmp);

			tmp[0] = para[1][i].dx;
			tmp[1] = para[1][i].dy;
			tmp[2] = para[1][i].dz;
			n.append(tmp);
			
			tmp[0] = para[2][i].dx;
			tmp[1] = para[2][i].dy;
			tmp[2] = para[2][i].dz;
			t.append(tmp);
		}

		Json::Value pp;
		pp["para1"] = p;
		pp["para2"] = n;
		pp["para3"] = t;

		Json::Value result = this->CallMethod("jsontest",pp);
		/* if (result.isInt()) { */
		/* 	return result.asInt(); */
		/* } else */
		/* 	throw jsonrpc::JsonRpcException(jsonrpc::Errors::ERROR_CLIENT_INVALID_RESPONSE, result.toStyledString()); */
        }
        void notifyServer() throw (jsonrpc::JsonRpcException)
        {
		Json::Value p;
		p = Json::nullValue;
		this->CallNotification("notifyServer",p);
        }
};

#endif /* _CLIENT_H_ */
