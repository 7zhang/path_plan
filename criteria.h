#ifndef _CRITERIA_H_
#define _CRITERIA_H_

class criteria {
public:
	virtual double operator() (double x);
	virtual void set_para (double para);
};

#endif /* _CRITERIA_H_ */
