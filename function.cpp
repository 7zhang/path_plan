#include "robotdata.h"
#include "robotkinematic.h"

//************************************
// Method:    PositiveRobot
// FullName:  PositiveRobot
// Access:    public 
// Returns:   extern "C"
// Qualifier: UINT KS_PositiveRobot(const JAngle& JA,TRANS& t6)
// Parameter: 昆山一号机器人正解
// Time:      2011/3/29
// Author:    葛浏昊
// 东南大学智能机器人实验室
//************************************
bool KS_PositiveRobot(const JAngle& JA,TRANS& t6)
{
	TRANS temp;
	double s1=sin(JA.angle[0]*PI/180.0);
	double c1=cos(JA.angle[0]*PI/180.0);
	double s2=sin(JA.angle[1]*PI/180.0);
	double c2=cos(JA.angle[1]*PI/180.0);
	double s3=sin(JA.angle[2]*PI/180.0);
	double c3=cos(JA.angle[2]*PI/180.0);
	double s4=sin(JA.angle[3]*PI/180.0);
	double c4=cos(JA.angle[3]*PI/180.0);
	double s5=sin(JA.angle[4]*PI/180.0);
	double c5=cos(JA.angle[4]*PI/180.0);
	double s6=sin(JA.angle[5]*PI/180.0);
	double c6=cos(JA.angle[5]*PI/180.0);     
    double s23=s2*c3+c2*s3;
	double c23=c2*c3-s2*s3;

	temp.rot.mem[0][0] = s1*(c5*c6*s4+c4*s6)+c1*(c4*c5*c6*s23+c23*c6*s5-s23*s4*s6);
	temp.rot.mem[1][0] = c6*(c4*c5*s1*s23-c1*c5*s4+c23*s1*s5)-(c1*c4+s1*s23*s4)*s6;
	temp.rot.mem[2][0] = -c6*s23*s5+c23*(c4*c5*c6-s4*s6);
	temp.rot.mem[0][1] = c6*(c4*s1-c1*s23*s4)-s6*(c5*s1*s4+c1*(c4*c5*s23+c23*s5));
	temp.rot.mem[1][1] = c1*(-c4*c6+c5*s4*s6)-s1*(c6*s23*s4+(c4*c5*s23+c23*s5)*s6);
	temp.rot.mem[2][1] = s23*s5*s6-c23*(c6*s4+c4*c5*s6);
	temp.rot.mem[0][2] = temp.rot.mem[1][0]*temp.rot.mem[2][1]-temp.rot.mem[2][0]*temp.rot.mem[1][1];    
	temp.rot.mem[1][2] = temp.rot.mem[2][0]*temp.rot.mem[0][1]-temp.rot.mem[0][0]*temp.rot.mem[2][1];
	temp.rot.mem[2][2] = temp.rot.mem[0][0]*temp.rot.mem[1][1]-temp.rot.mem[1][0]*temp.rot.mem[0][1];
	temp.pos.dx = c1*(KUNSHANROBOT_A2+c2*KUNSHANROBOT_A3+c23*(KUNSHANROBOT_D4+c5*KUNSHANROBOT_D5))
		          -KUNSHANROBOT_D5*(c1*c4*s23+s1*s4)*s5;
	temp.pos.dy = (KUNSHANROBOT_A2+c2*KUNSHANROBOT_A3+c23*(KUNSHANROBOT_D4+c5*KUNSHANROBOT_D5))*s1
		          -KUNSHANROBOT_D5*(c4*s1*s23-c1*s4)*s5;
	temp.pos.dz = KUNSHANROBOT_D1-KUNSHANROBOT_A3*s2-(KUNSHANROBOT_D4+c5*KUNSHANROBOT_D5)*s23
		          -c23*c4*KUNSHANROBOT_D5*s5;
	
	t6 = temp;    
	return 0;
}

//************************************
// Method:    InverseRobot
// FullName:  InverseRobot
// Access:    public 
// Returns:   extern "C"
// Qualifier: UINT KS_InverseRobot(const JAngle& JA,TRANS& t6)
// Parameter: 昆山一号机器人逆解
// Time:      2011/3/29
// Author:    葛浏昊
// 东南大学智能机器人实验室
//************************************
bool KS_InverseRobot(JAngle& Jointangle,const JAngle& lastJointangle,const TRANS& t6)
{
	JAngle joint_temp;
	TRANS trans_temp; 
    trans_temp = t6;   
	//求angle1
	double s1=trans_temp.pos.dy-KUNSHANROBOT_D5*trans_temp.rot.mem[1][2];  
	double c1=trans_temp.pos.dx-KUNSHANROBOT_D5*trans_temp.rot.mem[0][2];
	double temp11,temp12,temp1;
	double tan1=s1/c1;
	
	if(fabs(tan1)<1e-4)
	{
          if(fabs(lastJointangle.angle[0])<fabs(lastJointangle.angle[0]-180))
			
		{
			  temp1=0;
		}
		else
		{
			  temp1=PI;
		}
	}
	else
	{
		temp11=atan(tan1);
		if(temp11>0) 
		{
			temp12=temp11-PI;
		}
		else
		{
			temp12=temp11+PI;   	
		}
    	if (fabs(temp11-PI*lastJointangle.angle[0]/180)<fabs(temp12-PI*lastJointangle.angle[0]/180))
		{
		    temp1=temp11;
		}
	    else
		{
		    temp1=temp12;
		}  
	}
	//求angle3 ,当angle3超过180度时该算法有问题
	double k1=KUNSHANROBOT_D5*(cos(temp1)*trans_temp.rot.mem[0][2]+sin(temp1)*trans_temp.rot.mem[1][2])
		                 -cos(temp1)*trans_temp.pos.dx-sin(temp1)*trans_temp.pos.dy+KUNSHANROBOT_A2;
	double k2=trans_temp.pos.dz-KUNSHANROBOT_D5*trans_temp.rot.mem[2][2]-KUNSHANROBOT_D1;
	double k3=k1*k1+k2*k2-KUNSHANROBOT_A3*KUNSHANROBOT_A3-KUNSHANROBOT_D4*KUNSHANROBOT_D4;
	double c3=k3/(2*KUNSHANROBOT_D4*KUNSHANROBOT_A3);
	double temp3,temp31=0;	

	if((c3-1)>=1e-4)
	{
		return 1;
	}
	else
	{
		if (c3>1)
		{
			c3=1.0;
		}
	}
	
	temp3=acos(c3);
	temp31=-temp3;
	if (fabs(temp3-PI*lastJointangle.angle[2]/180)>fabs(temp31-PI*lastJointangle.angle[2]/180))
	{
		temp3=temp31;
	}
	//求angle2
	double a=-KUNSHANROBOT_D4*cos(temp3)-KUNSHANROBOT_A3;
	double b=KUNSHANROBOT_D4*sin(temp3);
	double k=(b*k1+a*k2)/(a*k1-b*k2);
	double temp2;
	temp2=atan2((b*k1+a*k2),(a*k1-b*k2));
	/*double temp2;//,temp21;
	if(fabs(k)<1e-5)
	{
		temp2=0;
	}
	else
	{
		temp2=atan(k);
		if(temp2>0)
		{
			temp21=temp2-PI;
		}
		else
		{
			temp21=temp2+PI;
		}
        if (fabs(temp21-PI*lastrobotangle.Angle2/180)<fabs(temp2-PI*lastrobotangle.Angle2/180))
		{
		     temp2=temp21;
	    }
	}*/
	//求angle4和angle5
	double test1=-sin(temp1)*trans_temp.rot.mem[0][2]+cos(temp1)*trans_temp.rot.mem[1][2];
	double test2=-sin(temp2+temp3)*(cos(temp1)*trans_temp.rot.mem[0][2]+sin(temp1)*trans_temp.rot.mem[1][2])-cos(temp2+temp3)*trans_temp.rot.mem[2][2];
	double temp41,temp42;
	double temp4,temp5;
	if ((fabs(test1)<1e-4)&&(fabs(test2)<1e-4))	   //奇异解
	{
		temp4=PI*lastJointangle.angle[3]/180;
	}
	else
	{
		temp41=atan(test1/test2);
		if(temp41>0)  
		{
			temp42=temp41-PI;
		}
		else
		{
			temp42=temp41+PI;
		}
		if (fabs(temp41-PI*lastJointangle.angle[3]/180)<fabs(temp42-PI*lastJointangle.angle[3]/180))
		{
			temp4=temp41;
		}
		else
		{
			temp4=temp42;
	}
	}
	double s5=-cos(temp4)*(trans_temp.rot.mem[2][2]*cos(temp2+temp3)+(trans_temp.rot.mem[0][2]*cos(temp1)+trans_temp.rot.mem[1][2]*sin(temp1))*sin(temp2+temp3))
		      +(trans_temp.rot.mem[1][2]*cos(temp1)-trans_temp.rot.mem[0][2]*sin(temp1))*sin(temp4);
	double c5=cos(temp2+temp3)*(trans_temp.rot.mem[0][2]*cos(temp1)+trans_temp.rot.mem[1][2]*sin(temp1))-trans_temp.rot.mem[2][2]*sin(temp2+temp3);
	/*if(fabs(s5)<1e-5 && fabs(c5)<1e-5)  //可删
	{
		temp5=PI*lastrobotangle.Angle5/180;
	}
	else
	{
		temp5=atan2(s5,c5);			
	}*/
	temp5=atan2(s5,c5);	
	//求angle6
    double s6=cos(temp5)*(cos(temp4)*(trans_temp.rot.mem[0][1]*cos(temp1)+trans_temp.rot.mem[1][1]*sin(temp1))*sin(temp2+temp3)+(-trans_temp.rot.mem[1][1]*cos(temp1)
		+trans_temp.rot.mem[0][1]*sin(temp1))*sin(temp4))-trans_temp.rot.mem[2][1]*sin(temp2+temp3)*sin(temp5) 
		+cos(temp2+temp3)*(trans_temp.rot.mem[2][1]*cos(temp4)*cos(temp5)+(trans_temp.rot.mem[0][1]*cos(temp1)+trans_temp.rot.mem[1][1]*sin(temp1))*sin(temp5));
	s6=-s6;
	double c6=cos(temp5)*(cos(temp4)*(trans_temp.rot.mem[0][0]*cos(temp1)+trans_temp.rot.mem[1][0]*sin(temp1))*sin(temp2+temp3)+(-trans_temp.rot.mem[1][0]*cos(temp1)
		+trans_temp.rot.mem[0][0]*sin(temp1))*sin(temp4))-trans_temp.rot.mem[2][0]*sin(temp2+temp3)*sin(temp5)+cos(temp2+temp3)*(trans_temp.rot.mem[2][0]*cos(temp4)*cos(temp5)
		+(trans_temp.rot.mem[0][0]*cos(temp1)+trans_temp.rot.mem[1][0]*sin(temp1))*sin(temp5));
    /*double temp6,temp61,temp62;
	if(fabs(s6)<1e-5 && fabs(c6)<1e-5)    //可删
	{
		temp61=PI*lastrobotangle.Angle6/180;
	}
	else
	{
		temp61=atan2(s6,c6);
	}
	if(temp61>0)             //可删
	{
		temp62=temp61-2*PI;
	}
	else
	{
		temp62=temp61+2*PI;
	}
	if (fabs(temp61-PI*lastrobotangle.Angle6/180)<fabs(temp62-PI*lastrobotangle.Angle6/180))  //可删
	{
		temp6=temp61;
	}
	else
	{
		temp6=temp62;		
	}*/
	double temp6;
	temp6=atan2(s6,c6);
	
	//赋值
	joint_temp.angle[0]=temp1*180/PI;
	joint_temp.angle[1]=temp2*180/PI;
	joint_temp.angle[2]=temp3*180/PI;
	joint_temp.angle[3]=temp4*180/PI;
    joint_temp.angle[4]=temp5*180/PI;
	joint_temp.angle[5]=temp6*180/PI;

	Jointangle = joint_temp;
	return 0;
}


//************************************
// Method:    New InverseRobot
// FullName:  KSNew_InverseRobot
// Access:    public 
// Returns:   extern "C"
// Qualifier: UINT KSNew_InverseRobot(JResolution& resolution, const TRANS& t6) 返回1表示出错
// Parameter: 昆山一号机器人逆解
// Time:      2011/3/29
// Author:    葛浏昊
// 东南大学智能机器人实验室
//************************************
bool KSNew_InverseRobot(JResolution& resolution, const TRANS& t6)
{
	JAngle joint_temp[8];
	TRANS trans_temp; 
    trans_temp = t6;  
	int i,j;
	double nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz;
	nx = trans_temp.rot.mem[0][0]; ny = trans_temp.rot.mem[1][0]; nz = trans_temp.rot.mem[2][0];
	ox = trans_temp.rot.mem[0][1]; oy = trans_temp.rot.mem[1][1]; oz = trans_temp.rot.mem[2][1];
	ax = trans_temp.rot.mem[0][2]; ay = trans_temp.rot.mem[1][2]; az = trans_temp.rot.mem[2][2];
	px = trans_temp.pos.dx;        py = trans_temp.pos.dy;        pz = trans_temp.pos.dz;

	//求angle1
	double temp1[2];
	double fs1 = py - KUNSHANROBOT_D5*ay;  
	double fc1 = px - KUNSHANROBOT_D5*ax;
	temp1[0] = atan2( fs1,  fc1);
	temp1[1] = atan2(-fs1, -fc1);

	for(i=0; i<4; i++)
	{
		joint_temp[i].angle[0] = temp1[0];  //J1-J4 的关节角1
	}
	for(i=4; i<8; i++)
	{
		joint_temp[i].angle[0] = temp1[1];  //J5-J8 的关节角1
	}
	
	//求angle3 
	double temp3[4];
	double k2 = pz - KUNSHANROBOT_D5*az - KUNSHANROBOT_D1;
	double k33 = pow(k2,2) - pow(KUNSHANROBOT_A3,2) - pow(KUNSHANROBOT_D4,2);
	for(i=0; i<2; i++)
	{
		double k1 = KUNSHANROBOT_D5*(cos(temp1[i])*ax + sin(temp1[i])*ay) 
			        - cos(temp1[i])*px - sin(temp1[i])*py + KUNSHANROBOT_A2;
		double k3 = pow(k1,2) + k33;
		double c3 = k3 / (2*KUNSHANROBOT_D4*KUNSHANROBOT_A3);

		if((c3-1)>=1e-4 || (c3+1)<=-(1e-4))    //*****??*****
		{
			for(j=0; j<4; j++)
			{
				resolution.err[4*i+j] = 1;
			}
			continue;
			//return 1;
		}
		else
		{
			if (c3 > 1)
			{
				c3=1.0;
			}
			else if (c3 < -1)
			{
				c3=-1.0;
			}
		}
		double fs3 = sqrt( 1-pow(c3,2) );
		temp3[2*i]   = atan2( fs3, c3);
		temp3[2*i+1] = atan2(-fs3, c3);
		
		for(j=0; j<2; j++)
		{
			joint_temp[i*4+j].angle[2] = temp3[2*i];      //J1,J2,J5,J6 的关节角3
		}
		for(j=2; j<4; j++)
		{
			joint_temp[i*4+j].angle[2] = temp3[2*i+1];  //J3,J4,J7,J8 的关节角3
		}
	}

	//求angle2
	double temp2[4];
	for(i=0; i<4; i++)
	{
		if(resolution.err[2*i] != 0)
		{
			continue;
		}
		double a = -KUNSHANROBOT_D4*cos(temp3[i]) - KUNSHANROBOT_A3;
		double b = KUNSHANROBOT_D4*sin(temp3[i]);
		double k1 = KUNSHANROBOT_D5*(cos(temp1[i/2])*ax + sin(temp1[i/2])*ay) 
			        - cos(temp1[i/2])*px - sin(temp1[i/2])*py + KUNSHANROBOT_A2;
		double fs2 = b*k1+a*k2;
		double fc2 = a*k1-b*k2;
		if( fabs(fs2) < 1e-4 )
		{
			fs2 = 0;			//为防止出现180和-180的重解问题
		}
		temp2[i] = atan2( fs2 , fc2 ); 
		for(j=0; j<2; j++)
		{
			joint_temp[i*2+j].angle[1] = temp2[i];      //J1-J8 的关节角2
		}

		//求angle4
		double fs4 = -sin(temp1[i/2])*ax + cos(temp1[i/2])*ay;
		double fc4 = -sin(temp2[i]+temp3[i])*(cos(temp1[i/2])*ax+sin(temp1[i/2])*ay) - cos(temp2[i]+temp3[i])*az;
		if ( (fabs(fs4)<1e-4) && (fabs(fc4)<1e-4) )	   //奇异解，此时sin5=0
		{
			resolution.err[2*i] = 2;
			resolution.err[2*i+1] = 2;
		//	joint_temp[2*i].angle[3] = PI/4;    //J1,J3,J5,J7 的关节角4，假设sin5>0
		//	joint_temp[2*i+1].angle[3] = PI/4;  //J2,J4,J6,J8 的关节角4，假设sin5<0
		}
		else
		{
		/*	if( fabs(fs4) < 1e-5 )
			{
				fs4 = 0;			//为防止出现180和-180的重解问题
			}*/
			joint_temp[2*i].angle[3] = atan2( fs4,  fc4);    //J1,J3,J5,J7 的关节角4，假设sin5>0
			joint_temp[2*i+1].angle[3] = atan2(-fs4, -fc4);  //J2,J4,J6,J8 的关节角4，假设sin5<0
		}
	}

	for(i=0; i<8; i++)
	{
		if(resolution.err[i] != 0)
		{
			continue;
		}
		//求angle5
		double temp4 = joint_temp[i].angle[3];
		double temp23 = temp2[i/2]+temp3[i/2];
		double s5 = - cos(temp4)*(az*cos(temp23) + sin(temp23)*(ax*cos(temp1[i/4])+ay*sin(temp1[i/4])))
		            + sin(temp4)*(ay*cos(temp1[i/4])-ax*sin(temp1[i/4]));
		double c5 =  cos(temp23)*(ax*cos(temp1[i/4])+ay*sin(temp1[i/4])) - sin(temp23)*az;
		if( (i%2==0 && s5<0) || (i%2==1 && s5>0) )  //与上一步的假设不符(i为偶数时,s5>0; i为奇数时,s5<0)
		{
			resolution.err[i] = 1;
		}
		else
		{
			if( fabs(s5) < 1e-4 )
			{
				s5 = 0;			//为防止出现180和-180的重解问题
			}
			joint_temp[i].angle[4] = atan2(s5,c5);	//J1-J8 的关节角5
	
			//求angle6
			double temp5 = joint_temp[i].angle[4];
			double s6 = -cos(temp5)*(  cos(temp4) * ( sin(temp23)*(cos(temp1[i/4])*ox+sin(temp1[i/4])*oy) + cos(temp23)*oz )
				                      +sin(temp4) * ( sin(temp1[i/4])*ox-cos(temp1[i/4])*oy ) )
				        -sin(temp5)*( cos(temp23) * ( cos(temp1[i/4])*ox+sin(temp1[i/4])*oy ) - sin(temp23)*oz );
			double c6 =  cos(temp5)*(  cos(temp4) * ( sin(temp23)*(cos(temp1[i/4])*nx+sin(temp1[i/4])*ny) + cos(temp23)*nz )
									  +sin(temp4) * ( sin(temp1[i/4])*nx-cos(temp1[i/4])*ny ) )
						+sin(temp5)*( cos(temp23) * ( cos(temp1[i/4])*nx+sin(temp1[i/4])*ny ) - sin(temp23)*nz );
			if( fabs(s6) < 1e-4 )
			{
				s6 = 0;			//为防止出现180和-180的重解问题
			}
			joint_temp[i].angle[5] = atan2(s6,c6);	//J1-J8 的关节角6
		}
	}
	
	//单位转换，弧度转换为度
	for(i=0; i<8; i++)
		for(j=0; j<6; j++)
		{
			joint_temp[i].angle[j] *= (180/PI);
		}

	bool error=1;
	for(i=0; i<8; i++)
	{
		resolution.m_Resolution[i] = joint_temp[i];
		error = ( resolution.err[i]==1 ) && error;
	}
	if(error)        //所有解都出错，即无解
	{
		return 1;
	}

	return 0;
}
