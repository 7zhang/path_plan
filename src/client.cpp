#include <iostream>
#include <jsonrpccpp/client/connectors/httpclient.h>
#include <boost/thread/thread.hpp>
#include "client.h"
#include "job.h"
#include "geometric.h"
using namespace jsonrpc;
using namespace std;
int main(int argc, char *argv[])
{
//    HttpClient httpclient("http://223.3.63.147:8383");
    HttpClient httpclient("http://localhost:8383");
//    HttpClient httpclient("http://223.3.56.142:8383");
    path_plan_client c(httpclient);

    if (argc != 2) {
	    std::cout << "usage: client test1.pos" << std::endl;
	    return 0;
    }
    job myjob(argv[1]);
    int job_id = 0;
    try
    {
	    std::vector<std::vector<Vector3D> > p;
	    p.push_back(myjob.get_p());
	    p.push_back(myjob.get_n());
	    p.push_back(myjob.get_t());
	    for (int i = 0; i < 1; i++) {
		    job_id = c.start_new(p);
//		    c.notifyServer();
		    cout << "add job "<< job_id << endl;
	    }
    }
    catch (JsonRpcException e)
    {
        cerr << e.what() << endl;
    }

    try {
    	    std::pair<int, int> finish_rate(0, 0);

    	    while(finish_rate.second < myjob.get_size()) {
		    path_plan_client::sys_state state;
    		    finish_rate = c.get_finish_rate(job_id, -1, state);
    		    if (finish_rate.first < 0) {
    			    std::cout << "job " << job_id << " size " 
    				      << -finish_rate.first << ": can't be finished, stop at "
    				      << finish_rate.second << std::endl;
    			    break;
    		    }
    		    std::cout << "job " << job_id << " size " << finish_rate.first << ": " 
    			      << finish_rate.second << " finished " << std::endl;
		    std::cout << state.to_string() << std::endl;
    		    boost::this_thread::sleep( boost::posix_time::milliseconds(200) );
    	    }
    }
    catch (JsonRpcException e)
    {
        cerr << e.what() << endl;
    }

    // boost::this_thread::sleep( boost::posix_time::milliseconds(3000) );
    // try {
    // 	    int pop_size = 80;
    // 	    double weight = 0.832432;
    // 	    int ret = c.set_sys_parameter(job_id, "de_pop_size", &pop_size, 1);
    // 	    if (ret == 0) {
    // 		    std::cout << "de_pop_size changed to " << pop_size << std::endl;
    // 	    } else {
    // 		    std::cout << "can't set parameter, check parameter" << std::endl;
    // 	    }
    // }
    // catch (JsonRpcException e)
    // {
    //     cerr << e.what() << endl;
    // }
}
