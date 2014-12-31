#include <stdio.h>
#include <iostream>
#include <cmath>
#include <jsonrpccpp/server/connectors/httpserver.h>
#include "plan_server.h"
using namespace std;
using namespace jsonrpc;
int count = 0;

void path_plan_server::jsontest(const Json::Value &request, Json::Value &response)
{
	cout << request.toStyledString() << endl;
	if (request.isObject()) {
//		response = 0;
	}
	switch (request.type()) {
	case Json::nullValue:
		cout << "nullValue" << endl;
		break;
	case Json::intValue:
		cout << "intValue" << endl;
		break;
	case Json::uintValue:
		cout << "uintValue" << endl;
		break;
	case Json::realValue:
		cout << "realValue" << endl;
		break;
	case Json::stringValue:
		cout << "stringValue" << endl;
		break;
	case Json::arrayValue:
		cout << "arrayValue" << endl;
		break;
	case Json::objectValue:
		cout << "objectValue" << endl;
		break;
	case Json::booleanValue:
		cout << "booleanValue" << endl;
		break;
	}
	cout << count << endl;
	count++;
}

int main()
{
    jsonrpc::HttpServer httpserver(8383);
    path_plan_server s(httpserver);
    s.StartListening();
    getc(stdin);
    s.StopListening();
    return 0;
}

