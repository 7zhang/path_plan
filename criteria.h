#ifndef _CRITERIA_H_
#define _CRITERIA_H_

class cri {
public:
	virtual double operator() (double x) = 0;
	virtual void set_para (double para) = 0;
};

#endif /* _CRITERIA_H_ */
