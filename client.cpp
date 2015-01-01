#include <iostream>
#include <jsonrpccpp/client/connectors/httpclient.h>
#include "client.h"
#include "job.h"
#include "geometric.h"
using namespace jsonrpc;
using namespace std;
int main(int argc, char *argv[])
{
    HttpClient httpclient("http://223.3.63.147:8383");
    path_plan_client c(httpclient);

    if (argc != 2) {
	    std::cout << "usage: client test1.pos" << std::endl;
	    return 0;
    }
    job myjob(argv[1]);
    try
    {
	    std::vector<std::vector<Vector3D> > p;
	    p.push_back(myjob.get_p());
	    p.push_back(myjob.get_n());
	    p.push_back(myjob.get_t());
	    for (int i = 0; i < 1; i++) {
		    int ret = c.jsontest(p);
		    c.notifyServer();
		    cout << i << endl;
	    }
    }
    catch (JsonRpcException e)
    {
        cerr << e.what() << endl;
    }
}
