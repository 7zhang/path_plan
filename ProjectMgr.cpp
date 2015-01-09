///////////////////////////////////////////////////////////
//  ProjectMgr.cpp
//  Implementation of the Class ProjectMgr
//  Created on:      28-七月-2011 10:46:28
//  Original author: Cheney Wong
///////////////////////////////////////////////////////////
#include "StdAfx.h"
#include "../head/ProjectMgr.h"
#include <afxdlgs.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//********2015.1.8 杨明亮********************
//***********Code Begin**********************
#include "cd.h"
#include "log.h"
//***********Code   end**********************

ProjectMgr::ProjectMgr(){
	open_flag = false;
}



ProjectMgr::~ProjectMgr(){

}

int ProjectMgr::read_word(FILE* fp, char* word)
{
	int i=0, index=0;
	int flag = -1;			// -1表示未读到起始字符，0表示读到单词，flag=1时表示读取结束
	for(i=0; i<50; i++)	{	word[i] = 0;}
	
	char c;
	while ((c=fgetc(fp)) != EOF)
	{
		if(0==flag)	{
			if((' '!=c)&&('#'!=c)&&('\n'!=c)&&('\t'!=c)&&('='!=c)&&('"'!=c))
			{
				word[index++] = c;
			}
			else {
				flag = 1;
				word[index] = '\0';
				break;
			}
		}
		else
		{
			if((' '!=c)&&('#'!=c)&&('\n'!=c)&&('\t'!=c)&&('='!=c)&&('"'!=c))	
			{
				flag = 0;
				word[index++] = c;
			}
		}
	}
	if (-1 == flag)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


bool ProjectMgr::read_cefile()
{
	CString file_path = prj_path + "\\" + prj_name+".ce";
	FILE* fp;
	if (NULL == (fp = fopen(file_path, "r")))
	{
	//	::MessageBox(NULL,"无法打开cell文件！","",MB_ICONINFORMATION);
		return false;
	}
	else
	{
		char modelpath[50];
		char cfgpath[50];
		char temp[50];
		int i=0;
		int flag = -1;		
			// -1:初始态；0：读到BeginProject；1：读模型路径；2：读配置路径；3：读到EndProject。
		while (i++, i<50)
		{
			read_word(fp, temp);
			switch(flag)
			{
				case -1:
					if(0==strcmp(temp,"BEGINPROJECT"))	flag = 0;		// 进入读取配置数据状态
					break;
				case 2:
					if(0==strcmp(temp,"ENDPROJECT"))					// 发现结束标示
					{
						flag = 3;
					}	
					break;				
				default:
					if(0==strcmp(temp,"MODEL_FILE"))					// 开始读取模型文件
					{
						read_word(fp, modelpath);
						model_name = modelpath;
						flag++;
					}
					else if(0==strcmp(temp,"CFG_FILE"))					// 开始读取配置文件
					{
						read_word(fp, cfgpath);
						cfg_name = cfgpath;
						int pos = cfg_name.Find('.');
						if (-1 != pos)
						{
							cfg_name = cfg_name.Left(pos);
						}
					
						flag++;
					}
					break;	
			}
			if(3==flag)	break;
		}
		fclose(fp);
		if(flag!=3)	
		{
			::MessageBox(NULL,"读取cell文件内容失败！","",MB_ICONINFORMATION);
			return false;
		}
		return true;		
	}
}

bool ProjectMgr::create_prj(char*prj_path, char* prjname)
{
	this->prj_path = prj_path;
	this->prj_name = prjname;
	this->cfg_name = prjname;
	this->model_name = prjname;

	//@[[创建ce工程文件
	//=========================================================
	CString cefpath;
	cefpath.Format("%s\\%s.ce",prj_path,prjname);
	FILE* fp = fopen(((LPSTR)(LPCTSTR)cefpath),"w+");
	char begin[] = "#BEGINPROJECT\n";
	char end[] = "#ENDPROJECT";
	char mdl[100] = "MODEL_FILE=\"\0";
	char cfg[100] = "CFG_FILE=\"\0";
	char mdlends[] = ".SLDASM\"\n";
	char cfgends[] = ".xml\"\n";
	strcat(mdl,prjname);
	strcat(mdl,mdlends);
	strcat(cfg,prjname);	
	strcat(cfg,cfgends);
	fwrite(begin,sizeof(char),strlen(begin),fp);
	fwrite(mdl,sizeof(char),strlen(mdl),fp);
	fwrite(cfg,sizeof(char), strlen(cfg),fp);
	fwrite(end,sizeof(char),strlen(end),fp);	

	fclose(fp);
	//=========================================================
	//@end]]


	//@[[创建配置文件*.cfg
	//=========================================================
	XMLResolver xmlres;
	CString cfgpath;
	cfgpath.Format("%s\\%s.xml",prj_path,prjname);

	if (xmlres.init_solver())
	{
		xmlres.create_document((LPSTR)(LPCTSTR)cfgpath);
		xmlres.release_solver();
	}
	else
	{
		AfxMessageBox("初始化XMLResolver失败。");
		return false;
	}
	
	//=========================================================
	//@end]]

	return true;
}
//********2015.1.8 杨明亮******************
//***********Code Begin**********************
volumenode *left_node[4], *right_node[10];
cdinit cd_init;
cd2 collision_detection2;
cdfinish cd_finish;
//***********Code   end**********************


bool ProjectMgr::open_prj()
{
	//********2015.1.8 杨明亮********************
	//***********Code Begin**********************

	char szModuleFileName[_MAX_PATH]; 
	GetModuleFileName(AfxGetInstanceHandle(),szModuleFileName,_MAX_PATH);	// 获取DLL所在路径
	CString cd_path = szModuleFileName;
	int pos = cd_path.ReverseFind('\\');
	cd_path = cd_path.Left(pos);	
	CString backup = cd_path;
	cd_path = cd_path + "\\cd.dll";
	
	HINSTANCE hinstance = LoadLibrary(cd_path);
	
	
	
	if (NULL == hinstance)
	{
		printf("error\n");
		AfxMessageBox("load cd.dll error");
		return false;
	}
	
	cd_init = (cdinit)GetProcAddress(hinstance, "cd_init");
	collision_detection2 = (cd2)GetProcAddress(hinstance, "collision_detection2");
	cd_finish = (cdfinish)GetProcAddress(hinstance, "cd_finish");
	
	if(cd_init == NULL) {
		AfxMessageBox("cd_init null");
		return false;
	}
	
	if(collision_detection2 == NULL) {
		AfxMessageBox("collision_detection2 null");
		return false;
	}
	
	if(cd_finish == NULL) {
		AfxMessageBox("cd_finish null");
		return false;
	}
	
	cd_parameter p;
	p.max_length = 60;
	p.max_triangle = 5;
	// 	CFileDialog dlg_cd(TRUE,"STL","*.STL",OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,"STL file (*.STL)|*.STL ||",NULL);
	// 	if (IDOK == dlg_cd.DoModal())
	// 	{
	// 		CString stlpath = dlg_cd.GetPathName();
	// 		left_node = cd_init((LPSTR)(LPCTSTR)stlpath, &p);
	// 		if (left_node == NULL)
	// 		{
	// 			AfxMessageBox("load stl error");
	// 			return false;
	// 		}
	// 	}
	// 	else
	// 		return false;
	// 	CFileDialog dlg_cd2(TRUE,"STL","*.STL",OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,"STL file (*.STL)|*.STL ||",NULL);
	// 	if (IDOK == dlg_cd2.DoModal())
	// 	{
	// 		CString stlpath = dlg_cd2.GetPathName();
	// 		right_node = cd_init((LPSTR)(LPCTSTR)stlpath, &p);
	// 		if (right_node == NULL)
	// 		{
	// 			AfxMessageBox("load stl error");
	// 			return false;
	// 		}
	// 	}
	// 	else
	// 		return false;

// 	CFileDialog dlg_cd(TRUE,"STL","*.STL",OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,"STL file (*.STL)|*.STL ||",NULL);
// 	if (IDOK == dlg_cd.DoModal())
// 	{
// 		CString stlpath = dlg_cd.GetPathName();
// 		left_node = cd_init((LPSTR)(LPCTSTR)stlpath, &p);
// 		if (left_node == NULL)
// 		{
// 			AfxMessageBox("cd initialize error");
// 			return false;
// 		}
// 	}
// 	else
// 		return false;
// 	
// 	for(int i = 0; i < 4; ++i) {
// 		CFileDialog dlg_cd1(TRUE,"STL","*.STL",OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,"STL file (*.STL)|*.STL ||",NULL);
// 		if (IDOK == dlg_cd1.DoModal())
// 		{
// 			CString stlpath = dlg_cd1.GetPathName();
// 			left_node[i] = cd_init((LPSTR)(LPCTSTR)stlpath, &p);
// 			if (left_node[i] == NULL)
// 			{
// 				AfxMessageBox("cd initialize error");
// 				return false;
// 			}
// 		}
// 		else
// 			return false;
// 	}

	cd_path = backup;
	CString tmp = cd_path + "\\robot_stl\\kr16_lbase.STL";
	left_node[0] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (left_node[0] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_larm.STL";
	left_node[1] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (left_node[1] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_plate.STL";
	left_node[2] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (left_node[2] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_workpiece.STL";
	left_node[3] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (left_node[3] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_rotate.STL";
	right_node[0] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (right_node[0] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_updown.STL";
	right_node[1] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (right_node[1] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_1.STL";
	right_node[2] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (right_node[2] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_2.STL";
	right_node[3] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (right_node[3] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_3.STL";
	right_node[4] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (right_node[4] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_4.STL";
	right_node[5] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (right_node[5] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_5.STL";
	right_node[6] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (right_node[6] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_6.STL";
	right_node[7] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (right_node[7] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_7.STL";
	right_node[8] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (right_node[8] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

	tmp = cd_path + "\\robot_stl\\kr16_gun.STL";
	right_node[9] = cd_init((LPSTR)(LPCTSTR)tmp, &p);
	if (right_node[9] == NULL)
	{
		AfxMessageBox("cd initialize error");
		return false;
	}

// 	AfxMessageBox("begin right node");
// 
// 	for(i = 0; i < 10; ++i) {
// 		CFileDialog dlg_cd1(TRUE,"STL","*.STL",OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,"STL file (*.STL)|*.STL ||",NULL);
// 		if (IDOK == dlg_cd1.DoModal())
// 		{
// 			CString stlpath = dlg_cd1.GetPathName();
// 			right_node[i] = cd_init((LPSTR)(LPCTSTR)stlpath, &p);
// 			if (right_node[i] == NULL)
// 			{
// 				AfxMessageBox("cd initialize error");
// 				return false;
// 			}
// 		}
// 		else
// 			return false;
// 	}	
	//***********Code   end**********************

	CFileDialog dlg(TRUE,"ce","*.ce",OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,"Cell file (*.ce)|*.ce ||",NULL);
	if(IDOK == dlg.DoModal())
	{
		prj_path = WorkCell::instance()->get_prjpath();
		if (read_cefile())							// 读取工程文件，提取工程信息
		{
			CString old_path;
			old_path.Format(prj_path+"\\Models\\"+model_name);	
			CComBSTR old_PathName = old_path;
			SLD::instance()->CloseDoc(old_PathName);	// 关闭当前工程的装配体文件
		}

		CString path = dlg.GetPathName();
		int pos = path.ReverseFind('\\');

		prj_path = path.Left(pos);					// 提取工程路径
		WorkCell::instance()->set_prjpath(prj_path);
		prj_name = dlg.GetFileName();				// 提取工程名称

		pos = prj_name.Find('.');
		if (-1 != pos)
		{
			prj_name = prj_name.Left(pos);
		}
		WorkCell::instance()->set_prjname(prj_name);

		if (read_cefile())							// 读取工程文件，提取工程信息
		{// 打开模型文件并提取workcell模型
			if (open_sldModel())
			{
				WorkCell::instance()->clear_devices();
				MotionController::instance()->clearRobotCtrlList();

				if (extract_cell())
				{
					open_flag = true;
					return WorkCell::instance()->refresh();
				}
				else{
					return false;
				}
			}
			else{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
	return false;
}


bool ProjectMgr::open_sldModel()
{
	CString path;
	path.Format(prj_path+"\\Models\\"+model_name);	
	//---------------------------//打开三维模型文件//----------------------------------
	CComBSTR PathName = path;
	CComPtr<IDispatch>pDispatch;
	CComBSTR sDefaultConfiguration(L"Default");	
	long fileerror, filewarning;
	CComPtr<IModelDoc2> pModel;
	
	SLD::instance()->OpenDoc6(PathName, swDocASSEMBLY, swOpenDocOptions_Silent, sDefaultConfiguration, &fileerror, &filewarning, &pModel);
	if (pModel==NULL)
	{
		AfxMessageBox("未找到装配体文件!" + path);
		return false;
	}

// 	SLD::instance()->IActivateDoc3( PathName, FALSE, &fileerror, &pModel );		// 激活该文件
// 	if (pModel==NULL)
// 	{
// 		AfxMessageBox("激活文件失败!" + path);
// 		return false;
// 	}

	return true;
}


bool ProjectMgr::extract_cell()
{
	CString path = prj_path+"\\"+cfg_name+".xml";

	XMLResolver xmlres;
	xmlres.init_solver();

	//------------------------------------------------------
 	if ( xmlres.attach_solver((LPSTR)(LPCTSTR)path) )
	{
		if (add_devices(xmlres)){
			return add_relations(xmlres);
		}
		else{
			return false;
		}
	}
	else{
		return false;
	}
}
bool ProjectMgr::add_devices(XMLResolver& xmlres)
{
	xml::IXMLDOMNodeListPtr devlist = NULL;
	devlist = xmlres.get_nodes("device");

	int i, count=0;
	count = devlist->Getlength();

	xml::IXMLDOMNodePtr devnode = NULL;
	for (i=0; i<count; i++)
	{
		
		devnode = devlist->Getitem(i);
		
		Device* curdev = NULL;
		curdev = device_factory(xmlres,devnode);

		if (curdev)
		{
			DEVTYPE dt = curdev->get_type();
			int a;
			if (dt == TOOL)
			{
				CString content;
				xmlres.get_subNodeAttrText(devnode,"tool","rpy",content);
				RPY tool;
				StringToRpy(content,tool);
				static_cast<Tool*>(curdev)->set_tool(tool);
			}
			else if (dt == MACHINE)
			{
				a=2;
			}
			else if (dt == ROBOT)
			{
				a=3;
			}
			else
			{

			}
		}
		devnode = NULL;
	}

	if (devnode)	devnode.Release();
	if (devlist)	devlist.Release();

	return true;
}


bool ProjectMgr::add_machine(XMLResolver& xmlres, xml::IXMLDOMNodePtr node)
{
	return true;
}

bool ProjectMgr::add_robot(XMLResolver& xmlres, xml::IXMLDOMNodePtr node)
{
	return true;
}
bool ProjectMgr::add_tool(XMLResolver& xmlres, xml::IXMLDOMNodePtr node)
{
	return true;
}
bool ProjectMgr::add_fixture(XMLResolver& xmlres, xml::IXMLDOMNodePtr node)
{
	return true;
}
bool ProjectMgr::add_relations(XMLResolver& xmlres)
{
	xml::IXMLDOMNodeListPtr relationList = NULL;
	relationList = xmlres.get_nodes("relation");

	if (relationList)
	{
		int i, count=0;
		count = relationList->Getlength();
		xml::IXMLDOMNodePtr node = NULL;

		for (i=0; i<count; i++)
		{
			CString parent, child, loca;
			RPY rpy;
			node = relationList->Getitem(i);
			if (node)
			{
				xmlres.get_nodeAttrText(node,"parent",parent);
				xmlres.get_nodeAttrText(node,"child",child);
				xmlres.get_nodeAttrText(node,"pos",loca);
				StringToRpy(loca,rpy);

				if ( !WorkCell::instance()->add_relation( (LPCTSTR)parent,(LPCTSTR)child, rpy) )
				{
					AfxMessageBox("坑爹了！别人已经有正当关系了，不要瞎搞！");
				}
			}

		}
		if (node) node.Release();
		relationList.Release();
	}
	return true;
}


void  ProjectMgr::StringToRpy(CString& content, RPY& result)
{
	double drpy[6];
	for (int i=0; i<6; i++)
	{
		int pos = content.Find(',');
		CString temp;
		if(i==5 && pos == -1)
			temp = content;
		else
			temp = content.Left(pos);
		drpy[i] = atof((LPCTSTR)temp);
		int leftlen = content.GetLength()-pos-1;
		content = content.Right(leftlen);
	}
	
	result.pos.dx = drpy[0];
	result.pos.dy = drpy[1];
	result.pos.dz = drpy[2];
	result.orient.dx = drpy[3];
	result.orient.dy = drpy[4];
	result.orient.dz = drpy[5];
		
}

Device* ProjectMgr::device_factory(XMLResolver& xmlres, xml::IXMLDOMNodePtr devnode)
{
	Device* dev;
	CString content, basename, type;

	xmlres.get_nodeAttrText(devnode,"type",type);

	//////////////////////////////////////////////////////////////////////////
	if (type == "MACHINE")
	{
		dev = new Machine;
	}
	else if (type == "ROBOT")
	{
		dev = new Robot;
	}
	else if (type == "TOOL")
	{
		dev = new Tool;
	}
	else if (type == "FIXTURE")
	{
		dev = new Fixture;
	}
	else
	{
		AfxMessageBox("又坑爹了？！找到一个不知道啥玩意的设备，加不了！");
		return NULL;
	}


	//////////////////////////////////////////////////////////////////////////
	//// 添加base组件
	ComponentX base;
	RPY loca;
	xmlres.get_subNodeAttrText(devnode,"base","name",basename);
	xmlres.get_subNodeAttrText(devnode,"location","rpy",content);
	StringToRpy(content,loca);
	
	if (!base.create((LPSTR)(LPCTSTR)basename,loca))
	{
		AfxMessageBox("创建componentX失败！");
		return NULL;
	}
	else
	{
		xmlres.get_nodeAttrText(devnode,"name",content);

		string st((LPCTSTR)type);
		dev->create(st, (LPSTR)(LPCTSTR)content, base,loca);
		
		//////////////////////////////////////////////////////////////////////////
		////添加关节
		xml::IXMLDOMNodeListPtr jolist = NULL;
		xml::IXMLDOMNodePtr josnode = NULL;
		int i, count=0;

		if (josnode = devnode->selectSingleNode("joints"))
		{
			jolist = josnode->selectNodes("joint");
			count = jolist->Getlength();
			
			for (i=0; i<count; i++)
			{
				xml::IXMLDOMNodePtr node=NULL;
				node = jolist->Getitem(i);
				Joint* jo = NULL;
				jo = get_joint(xmlres,node);
				if (!jo)
				{
					return NULL;
				}
				dev->add_joint(jo);
			}	
		}

		
		//////////////////////////////////////////////////////////////////////////
		////添加到工作单元中
		WorkCell* cell = WorkCell::instance();
		cell->add_device(dev);
		
		if (jolist)	jolist.Release();
		
		return dev;
	}
}

Joint* ProjectMgr::get_joint(XMLResolver& xmlres, xml::IXMLDOMNodePtr& node)
{
	Joint* jo = new Joint;
	CString content;
	
	//////////////////////////////////////////////////////////////////////////
	////添加pos以及name，即创建link
	xmlres.get_subNodeAttrText(node,"pose","rpy",content);
	RPY loc;
	StringToRpy(content,loc);		
	
	xmlres.get_subNodeAttrText(node,"link","name",content);
	if (!jo->create((LPSTR)(LPCTSTR)content,loc))
	{
		delete jo;
		return NULL;
	}
	
	//////////////////////////////////////////////////////////////////////////
	////设置类型type和运动方向dir
	Joint::JointType mtp;
	Joint::MotionDir mdir;
	xmlres.get_subNodeAttrText(node,"motype","tag",content);
	if (content == "LINEAR")	mtp = Joint::JointType::LINEAR;
	else	mtp = Joint::JointType::ROTATE;
	
	xmlres.get_subNodeAttrText(node,"motdir","tag",content);
	if (content == "DIR_X")		mdir = Joint::MotionDir::DIR_X;
	else if (content == "DIR_Y")		mdir = Joint::MotionDir::DIR_Y;
	else	mdir = Joint::MotionDir::DIR_Z;
	
	jo->setting(mtp,mdir);
	
	//////////////////////////////////////////////////////////////////////////
	////设置运动范围及初始值
	xmlres.get_subNodeAttrText(node,"range","min",content);
	double min = atof((LPCTSTR)content);
	xmlres.get_subNodeAttrText(node,"range","max",content);
	double max = atof((LPCTSTR)content);
	xmlres.get_subNodeAttrText(node,"init","value",content);
	double value = atof((LPCTSTR)content);
	xmlres.get_subNodeAttrText(node,"home","value",content);
	double home = atof((LPCTSTR)content);
	jo->set_range(max,min);
	jo->set_value(value);
	jo->set_home(home);
	
	return jo;
}

bool ProjectMgr::save_prj()
{
	CComPtr<IModelDoc2>pModelDoc = 0;
	SLD::instance()->get_IActiveDoc2(&pModelDoc);

	if (pModelDoc)
	{
		//-----------------保存sld文档-------------------------------
		HRESULT err, warning;
		VARIANT_BOOL res;
		pModelDoc->Save3(swSaveAsOptions_Silent,&err,&warning,&res);

		//-------------------保存xml文档-----------------------------
		save_cfgfile();	

		return true;
	}
	else
	{
		return false;
	}

}

bool ProjectMgr::save_cfgfile()
{
	XMLResolver xmlres;
	CString cfgname;
	cfgname.Format(prj_path+"\\"+cfg_name+".xml");
	if (xmlres.init_solver())
	{
		
		if ( xmlres.create_document((LPSTR)(LPCTSTR)cfgname) )
		{
			xmlres.addRoot("workcell");
			save_devices(xmlres);
			save_relationship(xmlres);
			xmlres.save();
		}
		return true;
	}
	else
	{
		return false;
	}
}

bool ProjectMgr::save_devices(XMLResolver& xmlres)
{
	xml::IXMLDOMNodePtr devs = xmlres.addNodeAtRoot("devices");
	
	DevList devlist = WorkCell::instance()->get_devlist();
	DevList::iterator itcur = devlist.begin();

	for (; itcur!=devlist.end(); itcur++)
	{
		xml::IXMLDOMNodePtr curdev = xmlres.addSubNode(devs,"device");
		xmlres.addNodeAttr(curdev,"name",(*itcur)->get_name());
		
		string type;
		if ( (*itcur)->get_type()== ROBOT)
			type = "ROBOT";
		else if ((*itcur)->get_type()==MACHINE)
			type = "MACHINE";
		else if ( (*itcur)->get_type()==TOOL)
			type = "TOOL";
		else if ( (*itcur)->get_type()==FIXTURE)
			type = "FIXTURE";
			
		xmlres.addNodeAttr(curdev,"type",type.c_str());

			// add node "base"
		xml::IXMLDOMNodePtr subnode = xmlres.addSubNode(curdev,"base");
		xmlres.addNodeAttr(subnode,"name",(*itcur)->get_basename());
		
			// add node "location"
		CString pos;
		TRANS trans = (*itcur)->get_refTrans();
		RPY rpy;
		trans.Trans2RPY(rpy);
		pos.Format("%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",rpy.pos.dx,rpy.pos.dy,rpy.pos.dz,
				rpy.orient.dx,rpy.orient.dy,rpy.orient.dz);
	
		subnode = xmlres.addSubNode(curdev,"location");
		xmlres.addNodeAttr(subnode,"rpy",(LPCTSTR)pos);

			// add joints
		if ( ((*itcur)->get_type()==ROBOT)||((*itcur)->get_type()==MACHINE) )
		{
			xml::IXMLDOMNodePtr joints = xmlres.addSubNode(curdev,"joints");

			for (int k=0; k<(*itcur)->get_jointNum(); k++)
			{
				Joint* jo = (*itcur)->get_joint(k+1);
				xml::IXMLDOMNodePtr jonode = xmlres.addSubNode(joints,"joint");
					
				//add node "link"
				subnode = xmlres.addSubNode(jonode,"link");
				xmlres.addNodeAttr(subnode,"name",jo->get_linkName());

				//add node "pose"
				CString pos;
				TRANS trans = jo->get_refTrans();
				RPY rpy;
				trans.Trans2RPY(rpy);
				pos.Format("%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",rpy.pos.dx,rpy.pos.dy,rpy.pos.dz,
					rpy.orient.dx,rpy.orient.dy,rpy.orient.dz);
				
				subnode = xmlres.addSubNode(jonode,"pose");
				xmlres.addNodeAttr(subnode,"rpy",(LPCTSTR)pos);

				// add node "motype"
				subnode = xmlres.addSubNode(jonode,"motype");
				if ( jo->get_type() == Joint::JointType::LINEAR)
					xmlres.addNodeAttr(subnode,"tag","LINEAR");
				else
					xmlres.addNodeAttr(subnode,"tag","ROTATE");
								
				// add node "motdir"
				subnode = xmlres.addSubNode(jonode,"motdir");
				string dir;
				Joint::MotionDir mdir = jo->get_dir();
				if ( Joint::MotionDir::DIR_X ==mdir )
					xmlres.addNodeAttr(subnode,"tag","DIR_X");
				else if ( Joint::MotionDir::DIR_Y ==mdir )
					xmlres.addNodeAttr(subnode,"tag","DIR_Y");
				else
					xmlres.addNodeAttr(subnode,"tag","DIR_Z");

				// add node "range"
				double minv, maxv;
				char cmin[20],cmax[20];
				jo->get_range(maxv,minv);				
				sprintf(cmin,"%.3f",minv);
				sprintf(cmax,"%.3f",maxv);
				subnode = xmlres.addSubNode(jonode,"range");
				xmlres.addNodeAttr(subnode,"min",cmin);
				xmlres.addNodeAttr(subnode,"max",cmax);

				// add node "init"
				double initx;
				char cinit[20];
				initx = jo->get_value();				
				sprintf(cinit,"%.3f",initx);
				subnode = xmlres.addSubNode(jonode,"init");
				xmlres.addNodeAttr(subnode,"value",cinit);

				// add node "home"
				double _home;
				char chome[20];
				_home = jo->get_home();				
				sprintf(chome,"%.3f",_home);
				subnode = xmlres.addSubNode(jonode,"home");
				xmlres.addNodeAttr(subnode,"value",chome);
			}
		}
		else if ( (*itcur)->get_type()==TOOL )
		{
			subnode = xmlres.addSubNode(curdev,"tool");
			Tool* to = (Tool*)(*itcur);
			RPY tr;
			to->get_tool().Trans2RPY(tr);
			CString temp;
			temp.Format("%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",tr.pos.dx, tr.pos.dy, tr.pos.dz,
				tr.orient.dx, tr.orient.dy, tr.orient.dz);
			xmlres.addNodeAttr(subnode,"rpy",(LPCTSTR)temp);
		}

	}
	return true;
}

bool ProjectMgr::save_relationship(XMLResolver& xmlres)
{
	xml::IXMLDOMNodePtr relts = xmlres.addNodeAtRoot("relationship");
	
	DevList devlist = WorkCell::instance()->get_devlist();
	DevList::iterator itcur = devlist.begin();

	for (; itcur!=devlist.end(); itcur++)
	{
		if ( (*itcur)->get_child()!=NULL )
		{
			xml::IXMLDOMNodePtr currel = xmlres.addSubNode(relts,"relation");
			xmlres.addNodeAttr(currel,"parent",(*itcur)->get_name());
			xmlres.addNodeAttr(currel,"child",(*itcur)->get_child()->get_name());
			CString pos;
			TRANS trans = (*itcur)->get_child()->get_refTrans();
			RPY rpy;
			trans.Trans2RPY(rpy);
			pos.Format("%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf",rpy.pos.dx,rpy.pos.dy,rpy.pos.dz,
				rpy.orient.dx,rpy.orient.dy,rpy.orient.dz);
			xmlres.addNodeAttr(currel,"pos",(LPCTSTR)pos);
		}
	}
	return true;
}