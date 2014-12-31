#ifndef _JSON_CLASS_H_
#define _JSON_CLASS_H_

class path_plan {
	int start_job(int system_id, int job_id);
	int add_job(int system_id, job j);
	int get_result(int job_id);
	
};

#endif /* _JSON_CLASS_H_ */
