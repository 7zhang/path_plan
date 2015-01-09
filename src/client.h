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

        int start_new(std::vector<std::vector<Vector3D> >& para) throw (jsonrpc::JsonRpcException)
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

		Json::Value result = this->CallMethod("start_new",pp);
		if (result.isInt()) {
			return result.asInt();
		} else {
			throw jsonrpc::JsonRpcException(jsonrpc::Errors::ERROR_CLIENT_INVALID_RESPONSE,
							result.toStyledString());
		}
        }
        void notifyServer() throw (jsonrpc::JsonRpcException)
        {
		Json::Value p;
		p = Json::nullValue;
		this->CallNotification("notifyServer",p);
        }

	std::pair<int, int> get_finish_rate(int job_id) throw (jsonrpc::JsonRpcException)
        {
		Json::Value p;
		p["para1"] = job_id;
		Json::Value result = this->CallMethod("get_finish_rate", p);
//		cout << result.toStyledString() << endl;

		if (result.isObject()) {
			int size = result["size"].asInt();
			int finished = result["finished"].asInt();
			return make_pair(size, finished);
		} else {
			throw jsonrpc::JsonRpcException(jsonrpc::Errors::ERROR_CLIENT_INVALID_RESPONSE,
							result.toStyledString());
		}
        }

	int set_sys_parameter(int job_id, std::string para_name, void *para_value, int restart)
	{
		int tmp_i = 0;
		double tmp_d = 0.0;
		Json::Value para;
		para["para1"] = para_name;
		para["para3"] = restart;
		para["para4"] = job_id;
		Json::Value result;

		if (para_name == "de_pop_size" || para_name == "de_thread_nr") {
			tmp_i = *(int *)para_value;
			if (tmp_i < 0) {
				std::cerr << "can't set de_pop_size to " << tmp_i
					  << ", value illegal" << std::endl;
				return -1;
			}
			para["para2"] = tmp_i;
			result = this->CallMethod("set_sys_parameter_int",para);
		} else if (para_name == "de_weight" || para_name == "de_crossover") {
			tmp_d = *(double *)para_value;
			if (tmp_d < 0.0 || tmp_d > 1.0) {
				std::cerr << "can't set de_crossover to " << tmp_d
					  << ", value illegal" << std::endl;
				return -1;
			}
			para["para2"] = tmp_d;
			result = this->CallMethod("set_sys_parameter_double",para);
		} else {
			return -1;
		}

		if (result.isInt()) {
			return result.asInt();
		} else {
			throw jsonrpc::JsonRpcException(jsonrpc::Errors::ERROR_CLIENT_INVALID_RESPONSE,
							result.toStyledString());
		}
	}
};

#endif /* _CLIENT_H_ */
