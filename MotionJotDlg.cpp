// MotionJotDlg.cpp : implementation file
//

#include "stdafx.h"
#include "resource.h"
#include "MotionJotDlg.h"
#include "AssmTreeDlg.h"
#include "SetStepSizeDlg.h"
#include "MovetoDlg.h"
#include "log.h"
#include "cd.h"
#include "KR5ARC.h"
//#include "Transform.h"

extern cdinit cd_init;
extern cd2 collision_detection2;
extern cdfinish cd_finish;
extern volumenode *left_node[], *right_node[];

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CMotionJotDlg property page

IMPLEMENT_DYNCREATE(CMotionJotDlg, CPropertyPage)

CMotionJotDlg::CMotionJotDlg() : CPropertyPage(CMotionJotDlg::IDD)
{
	//{{AFX_DATA_INIT(CMotionJotDlg)
	m_actobj = _T("");
	m_jogInfor = _T("50.0mm -- 10.0deg");
	m_jointinfor = _T("");
	m_jointvalue = _T("");
	//}}AFX_DATA_INIT
	curDevice = 0;
	jogsize_Lin = 50;
	jogsize_Ang = 10;
}

CMotionJotDlg::~CMotionJotDlg()
{
}

void CMotionJotDlg::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CMotionJotDlg)
	DDX_Control(pDX, IDC_JOINT_LIST, m_jotlist);
	DDX_Text(pDX, IDC_ACTIVE_OBJ, m_actobj);
	DDX_Text(pDX, IDC_STEPSIZE, m_jogInfor);
	DDX_Text(pDX, IDC_JOINT_INFOR, m_jointinfor);
	DDX_Text(pDX, IDC_JOINT_VALUE, m_jointvalue);

	DDX_Control(pDX, IDC_ADD, m_btnAdd);
	DDX_Control(pDX, IDC_SUB, m_btnSub);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CMotionJotDlg, CPropertyPage)
	//{{AFX_MSG_MAP(CMotionJotDlg)
	ON_BN_CLICKED(IDC_SETOBJECT, OnSetobject)
	ON_BN_CLICKED(IDC_SET_STEPSIZE, OnSetStepsize)
// 	ON_BN_CLICKED(IDC_ADD, OnAdd)
// 	ON_BN_CLICKED(IDC_SUB, OnSub)
	ON_LBN_SELCHANGE(IDC_JOINT_LIST, OnSelchangeJointList)
	ON_BN_CLICKED(IDC_HOME, OnHome)
	ON_BN_CLICKED(IDC_MOVETO, OnMoveto)
	ON_MESSAGE(WM_BUTTONDOWN_MESSAGE,OnButtonDown)		//当按钮按下时响应
	ON_MESSAGE(WM_BUTTONUP_MESSAGE,OnButtonUp)			//当按钮弹起时响应
	ON_WM_TIMER()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CMotionJotDlg message handlers

void CMotionJotDlg::OnSetobject() 
{
	// TODO: Add your control notification handler code here
	CAssmTreeDlg treedlg;
	if (treedlg.DoModal() == IDOK)
	{
		Device* actdev = treedlg.getActiveMechanism();
		curDevice = actdev;
		if (actdev != NULL)
		{
			m_actobj = actdev->get_name();
			//laymgr.SetActiveDevice(actdev);

			// --------------------------------------------------
			// 删除之前的内容并添加新设备的轴
			int i = 0;
			int j = m_jotlist.GetCount();
			for (i=0;i < j;i++)
			{
				m_jotlist.DeleteString(0);
			}

			int k = actdev->get_jointNum();
			for ( i=0; i<actdev->get_jointNum(); i++)
			{
				CString temp;
				temp.Format("J-%d",i+1);
				m_jotlist.AddString(temp);
			}
			// ---------------------------------------------------
			
		}
	}
	UpdateData(FALSE);
}

void CMotionJotDlg::OnSetStepsize() 
{
	// TODO: Add your control notification handler code here
	CSetStepSizeDlg dlg;
	if (dlg.DoModal() == IDOK)
	{
		jogsize_Lin = dlg.m_sizeL;
		jogsize_Ang = dlg.m_sizeR;
		m_jogInfor.Format("%.1lfmm -- %.1lfdeg",jogsize_Lin,jogsize_Ang);
	}
	UpdateData(FALSE);
}

void CMotionJotDlg::OnSelchangeJointList() 
{
	// TODO: Add your control notification handler code here
	int index = m_jotlist.GetCurSel();
	if (index>LB_ERR)
	{
		Joint * curjo = 0;
		if (NULL != (curjo = curDevice->get_joint(index+1)) )
		{
			double joval, vmin, vmax;
			joval = curjo->get_value();
			curjo->get_range(vmax,vmin);

			m_jointinfor.Format("J-%d:[%.2f, %.2f]",index+1,vmin,vmax);
			m_jointvalue.Format("J-%d:<%.2f>",index+1,joval);
		}		
	}
	else
	{
		m_jointinfor.Empty();
	}

	UpdateData(FALSE);
}

void CMotionJotDlg::OnHome() 
{
	// TODO: Add your control notification handler code here
	if (curDevice!=NULL)
	{
		for (int i=0; i<curDevice->get_jointNum(); i++)
		{
			Joint* jo = curDevice->get_joint(i+1);
			jo->move_to(jo->get_home());
		}
		WorkCell::instance()->refresh();
		SLD::redraw();
	}
}

void CMotionJotDlg::OnMoveto() 
{
	// TODO: Add your control notification handler code here
	if (curDevice==NULL)
	{
		AfxMessageBox(_T("请选择示教设备！"));
		return;
	}
	CMovetoDlg dlg;
	dlg.setMode(CMovetoDlg::JOINT);

	double init[6]={0,0,0,0,0,0};
	int i=0;
	for(i=0; i<6; i++)
	{
		curDevice->get_jointValue(i+1, init[i]);
		init[i] = double(int(init[i]*100))/100;	//保留两位有效数字
	}
	dlg.setInit(init);

	if (dlg.DoModal() == IDOK)
	{
		if (curDevice!=NULL)
		{
			double* ang = dlg.getValue();
			for (int i=0; i<curDevice->get_jointNum(); i++)
			{
				Joint* jo = curDevice->get_joint(i+1);
				jo->move_to(*(ang+i));
			}
			WorkCell::instance()->refresh();
			SLD::redraw();
		}
	}
}

LRESULT CMotionJotDlg::OnButtonDown( WPARAM wParam, LPARAM lParam )
{
	UINT btnId[] = {IDC_ADD,IDC_SUB,NULL};
	UINT selBtnId = NULL;
	CPoint* point = (CPoint*) wParam;					//鼠标点击点在Screen下的坐标
	
	int i=0;
	for(i=0; btnId[i]!=NULL; i++)
	{
		CRect btnRect;
		GetDlgItem(btnId[i])->GetWindowRect(btnRect);	//按钮矩形在Screen下的坐标
		if( btnRect.PtInRect(*point) )					//判断点point是否在矩形btnRect内
		{
			selBtnId = btnId[i];
			break;
		}
	}
	if(selBtnId == NULL)
	{
		return 0;
	}
	switch(selBtnId)
	{
	case IDC_ADD:
		StartJog(true);
		break;
	case IDC_SUB:
		StartJog(false);
		break;
	}
	return 0;
}

LRESULT CMotionJotDlg::OnButtonUp( WPARAM wParam, LPARAM lParam )
{
	StopJog();
	return 0;
}

void CMotionJotDlg::OnTimer(UINT nIDEvent) 
{
	// TODO: Add your message handler code here and/or call default
	
	// nIDEvent = 1,2 用于Jog示教，m_JogElapse，m_JogElapse*5，m_JogElapse=100

	if(nIDEvent == 1)
	{
		RefreshJog(m_jogIndex, m_jogDir);
		if (curDevice->get_type()==ROBOT)
		{
			Robot* prob = static_cast<Robot*>(curDevice);
			JAngle ja = prob->get_angle();
			if (prob->get_kinematicAlg()== NULL)
			{
// 				StopJog();
// 				AfxMessageBox("请配置机器人.");
				CPropertyPage::OnTimer(nIDEvent);
				return;
			}
			illegal_joint jnum = prob->get_kinematicAlg()->IsLegal(ja);

			if (NOJOINT != jnum)
			{
				StopJog();
				CString temp;
				temp.Format("机器人关节 [%d] 超限！运动停止。",jnum);
				AfxMessageBox(temp);
			}	
		}
		Robot* rob = 0;
		rob = MotionController::instance()->getRobotController().getHostRobot();
		if ( rob!=NULL && rob->get_kinematicAlg()!=NULL)
		{
			//			TRANS gun1111111111111 = rob->get_wldTrans() * rob->get_endTrans();	// 机器人末端在世界坐标系中的变换矩阵 = 基座在世界坐标系中的变换矩阵 * 机器人末端相对于基座的变换矩阵
			// 			RPY rpypos111111111111 = MotionController::instance()->getRobotController().getWorkCoord();
			// 			TRANS part2222222222222222;
			// 			rpypos111111111111.RPY2Trans(part2222222222222222);
			JAngle extAngle = MotionController::instance()->getRobotController().getExtAngle();	
			JAngle robotAngle = MotionController::instance()->getRobotController().getHostRobot()->get_angle();
			KR5ARC_RKA *kr5 = new KR5ARC_RKA();
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 10; j++) {					
					TRANS left_trans = kr5->getTransWorldToWorkpiece(i, extAngle);
					log_trans("left_trans", left_trans);
					TRANS right_trans = kr5->get_trans_to_world(j, robotAngle, extAngle);
					log_trans("right_trans", right_trans);

					vector3d v1, v2;
					v1.x = left_trans.pos.dx;
					v1.y = left_trans.pos.dy;
					v1.z = left_trans.pos.dz;

					v2.x = right_trans.pos.dx;
					v2.y = right_trans.pos.dy;
					v2.z = right_trans.pos.dz;

					int result = collision_detection2(left_node[i], left_trans.rot.mem, &v1, right_node[j], right_trans.rot.mem, &v2);
					if (result == 1) {
					 	KillTimer(1);
					 				//		AfxMessageBox("xxx");
					 	CString s;
					 	s.Format("left: %d, right: %d", i, j);				
				 				
 						AfxMessageBox(s);
					}
				}
			}
		}

// 		if ( rob!=NULL && rob->get_kinematicAlg()!=NULL)
// 		{
// 			//			TRANS gun1111111111111 = rob->get_wldTrans() * rob->get_endTrans();	// 机器人末端在世界坐标系中的变换矩阵 = 基座在世界坐标系中的变换矩阵 * 机器人末端相对于基座的变换矩阵
// 			// 			RPY rpypos111111111111 = MotionController::instance()->getRobotController().getWorkCoord();
// 			// 			TRANS part2222222222222222;
// 			// 			rpypos111111111111.RPY2Trans(part2222222222222222);
// 			JAngle extAngle = MotionController::instance()->getRobotController().getExtAngle();	
// 			JAngle robotAngle = MotionController::instance()->getRobotController().getHostRobot()->get_angle();
// 						
// 			int rel = 0;
// 			for(int i = 0; i < 7; ++i) {
// 				tmp1.x = tran[i].pos.dx;
// 				tmp1.y = tran[i].pos.dy;
// 				tmp1.z = tran[i].pos.dz;
// 				
// 				result[i] = collision_detection2(left_node, part2222222222222222.rot.mem, &tmp2, right_node[i], tran[i].rot.mem, &tmp1);
// 				rel = rel || result[i];
// 			}			
// 			
// 			if (rel == 1)
// 			{
// 				KillTimer(1);
// 				//		AfxMessageBox("xxx");
// 				CString s;
// 				s.Format("axis: %d, %d, %d, %d, %d, %d, gun: %d, angle: 1 = %f, 2 = %f, 3 = %f, 4 = %f, 5 = %f, 6 = %f",
// 					result[0], result[1], result[2], result[3], result[4], result[5], result[6], rob->get_angle().angle[0], 
// 					rob->get_angle().angle[1], rob->get_angle().angle[2], rob->get_angle().angle[3], 
// 					rob->get_angle().angle[4], rob->get_angle().angle[5]);				
// 				
// 				AfxMessageBox(s);
// 			}
// 			
// 			KR5ARC rob;
// 			for (int i = 0; i < 4; i++) {
// 				for (int j = 0; j < 8; j++) {
// 					
// 				}
// 			}
// 		}		
	}

	CPropertyPage::OnTimer(nIDEvent);
}

void CMotionJotDlg::StartJog( const bool& JogDir )
{
	m_jogDir = JogDir;

	m_jogIndex = m_jotlist.GetCurSel();
	if (m_jogIndex >= 0)
	{
		int JogElapse = 100;
		SetTimer(1,60,NULL);		// 示教运动定时器
	}
	else
	{
		AfxMessageBox("注意啦：选一个能使的关节！");
	}
}

void CMotionJotDlg::RefreshJog( const int& nIndex, const bool& JogDir )
{
	if (nIndex < 0)
	{
		return;
	}
	int dir = 0;
	if (JogDir == true)		// Jog-Add
	{
		dir = 1;
	}
	else					// Jog-Sub
	{
		dir = -1;
	}

	double size = jogsize_Lin;
	if (curDevice->get_joint(nIndex+1)->get_type() == Joint::JointType::ROTATE)
	{
		size = jogsize_Ang;
	}
	
	curDevice->joint_jog(nIndex+1,dir*size);		
	curDevice->refresh(curDevice->get_wldTrans());
	
	double joval; 
	curDevice->get_jointValue(nIndex+1, joval);
	m_jointvalue.Format("J-%d:<%.2f>",nIndex+1,joval);
	UpdateData(FALSE);
	SLD::redraw();

}

void CMotionJotDlg::StopJog()
{
	KillTimer(1);
}
