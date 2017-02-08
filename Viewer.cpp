/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
// Undeprecate CRT functions
#ifndef _CRT_SECURE_NO_DEPRECATE 
#define _CRT_SECURE_NO_DEPRECATE 1
#endif

#include "Stdafx.h"
#include "Viewer.h"
#include "CommandHandling.h"
#include "time.h"

using namespace std;
using namespace Eigen;
#define PAI 3.14159

#if (ONI_PLATFORM == ONI_PLATFORM_MACOSX)
#include <GL/glut.h>
#else
#include <GL/glut.h>
#endif


#define GL_WIN_SIZE_X	1200
#define GL_WIN_SIZE_Y	1024
#define TEXTURE_SIZE	512

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_IMAGE

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

SampleViewer* SampleViewer::ms_self = NULL;
//FILE *fp;
int Switch1 = 0, Switch2 = 0, Switch3 = 0;
int Flag = 0;
vector <double> vertex1, vertex2, vertex3;
vector <double> normal1, normal2, normal3;
float touka = 0.0f;
double p, q, R, a, b, c;
float wheel = 1.00;
double mx = 0.0, my = 0.0;


//バネ質点モデル用
//2015.1.14現在
//筋肉を線として扱っている
//そのため連結は2点のみ
//↓はVolumeModelになったとき用
//頂点バッファ:　VB[]={x1,y1,z1,x2,y2,z2,…}
//頂点インデックスバッファ: IB[]={1,2,3, 3,5,6 }3つで一つの三角パッチの組みを生成
vector<double> VB;//頂点バッファ
vector<int> IB;//頂点インデックスバッファ
vector<double> Edge;//エッジ長
vector<int> TempConnection1;
vector<double> a_x; vector<double> a_y; vector<double> a_z;//accel
vector<double> v_x; vector<double> v_y; vector<double> v_z;//velocity
vector<double> f_x; vector<double> f_y; vector<double> f_z;//force

														   //パラメータ(自分で適当に調整．様子を見ながら)
const float K = 1;//ばね定数
const float D = 0.1;//減衰定数//おそらく０～１の範囲でなければならない。
const float M = 1;//質点の質量
const int LOOP = 100;
const float dt = 0.3;
const int Depth = 5;//探索深度の設定（13まで動作確認済み
vector<double> vb_x1, vb_x2, vb_y1, vb_y2, vb_z1, vb_z2;
struct quaternion
{
	double w;
	double x;
	double y;
	double z;
};
float p_q0, p_qx, p_qy, p_qz;
double Z;
// point to the command handling class
CCommandHandling *pCommandHandling;

bool m_bResetHardware;     // reset hardware variable
bool m_bSystemInitialized; // is the system initialized
bool m_bPortsActivated;    // are ports activated
						   //bool m_bUse0x0800Option;

int m_nCOMPort;	// the current com port number
int m_bWireless;
/***********
通常は、Line 61 をコメントアウトする
3Dトレセンの場合は、Line 61 を使用する
***********/

bool m_bStopTracking; // flag that tells the thread to stop tracking
bool m_bIsTracking;   // flag that specifies if we are tracking
int m_nTrackingMode;  // tracking mode is 0 for TX data, else for BX data.

int LoopMax = 1000;
int TimeMax = 1000;
//回転マトリックス
double Rotate[16];

quaternion Target;
quaternion current = { 1.0, 0.0, 0.0, 0.0 };
quaternion & operator *(quaternion &q1, quaternion &q2)
{
	quaternion q0 = {
		q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
		q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
		q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
		q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
	};
	q1 = q0;
	return q1;
}


//void SampleViewer::glutKeyboard(unsigned char key, int x, int y)
//{
//	SampleViewer::ms_self->onKey(key, x, y);


void SampleViewer::INITIALIZEDPOLARIS()
{
	SampleViewer::ms_self->initializedPolaris();
}

SampleViewer::SampleViewer()
{
	ms_self = this;
}
SampleViewer::~SampleViewer()
{
	ms_self = NULL;
}

/*****************************************************************
ResetSystem

Source: OnResetSystem()

Description:
　This routine handles what happens when the user
 　presses the Reset System button.  It resets the
  　system they are connected to.
   *****************************************************************/
void ResetSystem()
{
	int nRet = 0;
	/*
	* This feature is useful for debugging only
	*/
	ReadINIParm("Communication", "Reset Hardware", "0", &m_bResetHardware);

	// これは要らない？
	//m_bResetHardware = TRUE;

	ReadINIParm("Communication", "COM Port", "0", &m_nCOMPort);
	// 	ReadINIParm( "Communication", "COM Port", "4", &m_nCOMPort );

	ReadINIParm("Communication", "Wireless", "0", &m_bWireless);

	pCommandHandling->nCloseComPorts();
	if (!pCommandHandling->nOpenComPort(m_nCOMPort)) {
		/*
		MessageBox( "COM Port could not be opened.  "
		"Check your COM Port settings and\n"
		"make sure you system is turned on.",
		"COM ERROR", MB_ICONERROR|MB_SYSTEMMODAL|MB_SETFOREGROUND );
		return;
		*/

		cout << "In Reset Session, COM Port could not be opened.  "
			<< "Check your COM Port settings and\n"
			<< "make sure you system is turned on. \n",
			exit(1);
	}

	if (m_bResetHardware) {
		if (!pCommandHandling->nHardWareReset(m_bWireless))
			exit(1);

		if (!pCommandHandling->nSetSystemComParms(0, 0, 0, 0, 0))
			exit(1);
	}
}


/*****************************************************************
InitializeSystem

Source:
OnInitializeSystem

Description:
This routine handles what happen when the user
presses the Intialize System button.  We initialize
the System.
*****************************************************************/
void InitializeSystem(void)
{
	int nBaudRate = 0;
	int nStopBits = 0;
	int nParity = 0;
	int nDataBits = 0;
	int nHardware = 0;

	//CWaitCursor wait;

	// read the COM port parameters from the ini file
	// Section[Communication]から、変数[Baud Rate](Defalut=0)の値を、
	// サイズsizeof(nBaudRate)でnBaudRateに格納する
	ReadINIParm("Communication", "Baud Rate", "0", &nBaudRate);
	ReadINIParm("Communication", "Stop Bits", "0", &nStopBits);
	ReadINIParm("Communication", "Parity", "0", &nParity);
	ReadINIParm("Communication", "Data Bits", "0", &nDataBits);
	ReadINIParm("Communication", "Hardware", "0", &nHardware);
	ReadINIParm("Communication", "Wireless", "0", &m_bWireless);

	// Auroraでは、この関数はコメントアウト
	ReadINIParm("Communication", "COM Port", "0", &m_nCOMPort);


	// For 3D Training Seminar 
	// ReadINIParm("Communication", "COM Port", "4", sizeof(m_nCOMPort), &m_nCOMPort);  

	// This feature is useful for debugging only, m_bResetHardware is set to 
	// TRUE to disable it.
	ReadINIParm("Communication", "Reset Hardware", "0", &m_bResetHardware);
	m_bResetHardware = TRUE;
	// close, then open the port
	pCommandHandling->nCloseComPorts();

	if (!pCommandHandling->nOpenComPort(m_nCOMPort))
	{
		cout << "COM Port could not be opened. \n"
			<< "Check your COM Port settings and "
			<< "make sure you system is turned on. \n",
			exit(1);
	}



	// if we are supposed to reset, call the reset now
	if (m_bResetHardware)
	{
		if (!pCommandHandling->nHardWareReset(m_bWireless))
		{
			exit(1);
		}
	}

	// Get the timeout values for the commands
	// this will return an error with all other systems, other than Vicra
	pCommandHandling->CreateTimeoutTable();

	// set the System COM Port parameters, then the computers COM Port parameters.
	// if that is successful, initialize the system

	if (pCommandHandling->nSetSystemComParms(nBaudRate, nDataBits, nParity, nStopBits, nHardware))
	{
		if (pCommandHandling->nSetCompCommParms(nBaudRate, nDataBits, nParity, nStopBits, nHardware))
		{
			if (pCommandHandling->nInitializeSystem())
			{
				// get the system information
				if (!pCommandHandling->nGetSystemInfo())
				{
					// Check system type: Polaris, Polaris Accedo, and Aurora
					cout << "Could not determine type of system\n"
						<< "(POLARIS, POLARIS ACCEDO or AURORA)\n"
						<< "Please contact Northern Digital Inc. for assistance \n";
					return;
				}

				// Set firing rate if system type is Polaris or Polaris Accedo.				
				if (pCommandHandling->m_dtSystemInformation.nTypeofSystem != AURORA_SYSTEM)
				{
					pCommandHandling->nSetFiringRate();
				}

				m_bSystemInitialized = TRUE;
				cout << "System successfully intialized \n";
				return;
			}
			else
			{
				cout << "System could not be initialized. "
					<< "Check your COM Port settings, make sure your\n"
					<< "system is turned on and the system components are compatible.\n";
			}
		}
	}
}


/*****************************************************************
ActivePorts

Source:
nActivePorts

Return Value:
int - 1 if successful, 0 otherwise

Description:
This routine actives the ports plugged into the system
*****************************************************************/
int nActivatePorts()
{

	// if we can active the ports, we then fill the port information
	// on the main dialog
	if (pCommandHandling->nActivateAllPorts())
	{
		m_bPortsActivated = TRUE;

		if (!m_bIsTracking)
		{
			cout << "Ports successfully activated \n";
		}
		return 1;
	}

	m_bPortsActivated = FALSE;

	cout << "Ports could not be activated.\n";
	cout << "Check your SROM image file settings and \n";
	cout << "make sure your system is turned on and initialized.";
	return 0;
}


/*****************************************************************
ActivePorts

Source:
OnActivePorts

Description:
This routine handles what happens when the user presses
the active ports button.
*****************************************************************/
void ActivatePorts()
{
	nActivatePorts();

	/*
	if (!nActivatePorts())
	exit(1);
	*/
}


/*****************************************************************
StartTracking

Source:
nStartTracking

Return Value:
int - 1 if successful, 0 otherwise

Description:
This routine starts the System tracking
*****************************************************************/
int StartTracking()
{
	if (pCommandHandling->nStartTracking())
	{

		// if we can start tracking, set the appropriate window text,
		// start the tracking thread and set the mode.
		m_bIsTracking = TRUE;
		m_bStopTracking = FALSE;

		return 1;
	}
	return 0;
}


/*****************************************************************
GetSystemTransformData

Source:
GetSystemTransformData

Description:
This routine gets the next set of transformation data.
*****************************************************************/
int GetSystemTransformData(void)
{
	if (!m_bIsTracking)
		return 0;

	bool m_bTrackOOV = TRUE;

	// if tracking mode is 0, we are asking for TX data, else we are
	// asking for BX data.
	if (m_nTrackingMode == 0)
	{
		//if ( !pCommandHandling->nGetTXTransforms( m_bUse0x0800Option ? true : false ) )
		if (!pCommandHandling->nGetTXTransforms(m_bTrackOOV))
			return 0;
	}
	else if (m_nTrackingMode == 1)
	{
		//if ( !pCommandHandling->nGetBXTransforms( m_bUse0x0800Option ? true : false ) )	
		if (!pCommandHandling->nGetTXTransforms(m_bTrackOOV))
			return 0;
	}


	// if a new port has become occupied we do the following:
	// 1) Stop tracking
	// 2) Activate Ports
	// 3) Start Tracking
	if (pCommandHandling->m_dtSystemInformation.bPortOccupied)
	{
		if (pCommandHandling->nStopTracking() &&
			nActivatePorts() &&
			pCommandHandling->nStartTracking())
			return 1;

		// We don't want the tracking thread to track if 
		// activating the ports failed!
		m_bStopTracking = TRUE;
		m_bIsTracking = FALSE;
		return 0;
	}


	for (int i = 0; i < NO_HANDLES; i++)
	{
		if (pCommandHandling->m_dtHandleInformation[i].HandleInfo.bInitialized > 0 &&
			pCommandHandling->m_dtHandleInformation[i].szToolType[1] != '8')

		{
			// only update the frame if the handle isn't disabled
			if (pCommandHandling->m_dtHandleInformation[i].Xfrms.ulFlags == TRANSFORM_VALID ||
				pCommandHandling->m_dtHandleInformation[i].Xfrms.ulFlags == TRANSFORM_MISSING)
			{

			}

			// display port information
			/*
			if ( pCommandHandling->m_dtHandleInformation[i].Xfrms.ulFlags == TRANSFORM_VALID ) {

			cout << "Port No. " << pCommandHandling->m_dtHandleInformation[i].szPhysicalPort << endl;
			cout << "Translation ( "
			<< pCommandHandling->m_dtHandleInformation[i].Xfrms.translation.x << " "
			<< pCommandHandling->m_dtHandleInformation[i].Xfrms.translation.y << " "
			<< pCommandHandling->m_dtHandleInformation[i].Xfrms.translation.z << " ) \n";

			cout << "Rotation ( "
			<< pCommandHandling->m_dtHandleInformation[i].Xfrms.rotation.q0 << " "
			<< pCommandHandling->m_dtHandleInformation[i].Xfrms.rotation.qx << " "
			<< pCommandHandling->m_dtHandleInformation[i].Xfrms.rotation.qy << " "
			<< pCommandHandling->m_dtHandleInformation[i].Xfrms.rotation.qz << " ) \n";

			cout << "Error = " << pCommandHandling->m_dtHandleInformation[i].Xfrms.fError << endl;
			if ( pCommandHandling->m_dtHandleInformation[i].HandleInfo.bPartiallyOutOfVolume )
			cout << "POOV \n";
			else if ( pCommandHandling->m_dtHandleInformation[i].HandleInfo.bOutOfVolume )
			cout << "OOV \n";
			else
			cout << "OK \n";
			}
			else {
			if ( pCommandHandling->m_dtHandleInformation[i].Xfrms.ulFlags == TRANSFORM_MISSING )
			cout << "MISSING \n";
			else {
			cout << "DISABLED \n";
			}
			if ( pCommandHandling->m_dtHandleInformation[i].HandleInfo.bPartiallyOutOfVolume )
			cout << "POOV \n";
			else if ( pCommandHandling->m_dtHandleInformation[i].HandleInfo.bOutOfVolume )
			cout << "OOV \n";
			else
			cout << "--- \n";

			}
			*/
		}
	}
	return 1;
}


/*****************************************************************
GetTrackingData

Source:
nGetSystemTransformData

Return Value:
int - 1 if successful, 0 otherwise

Description:
This routine stops the tracking procedure.
*****************************************************************/
void SampleViewer::GetTrackingData(float &p_q0, float &p_qx, float &p_qy, float &p_qz, float &t_x, float &t_y, float &t_z, float &t_x2, float &t_y2, float &t_z2)
{



	GetSystemTransformData();
	for (int j = 0; j<3; j++)
	{
		if (pCommandHandling->m_dtHandleInformation[j].Xfrms.ulFlags == TRANSFORM_VALID)
		{

			t_x = pCommandHandling->m_dtHandleInformation[1].Xfrms.translation.y;
			t_y = pCommandHandling->m_dtHandleInformation[1].Xfrms.translation.x;
			t_z = pCommandHandling->m_dtHandleInformation[1].Xfrms.translation.z;

			t_x2 = pCommandHandling->m_dtHandleInformation[2].Xfrms.translation.y;
			t_y2 = pCommandHandling->m_dtHandleInformation[2].Xfrms.translation.x;
			t_z2 = pCommandHandling->m_dtHandleInformation[2].Xfrms.translation.z;

			p_q0 = -1 * pCommandHandling->m_dtHandleInformation[j].Xfrms.rotation.q0;
			p_qx = -1 * pCommandHandling->m_dtHandleInformation[j].Xfrms.rotation.qy;
			p_qy = -1 * pCommandHandling->m_dtHandleInformation[j].Xfrms.rotation.qx;
			p_qz = -1 * pCommandHandling->m_dtHandleInformation[j].Xfrms.rotation.qz;
			//current_time);
			//}
		}
		else {
			//	current_time = timeGetTime();          
			//for (int k=0; k<7; k++)
			//fprintf(fp1,"MISSING ");
			//fprintf(fp1,"%d \n",current_time);
			//}
		}
	}
	//	}

}



/*****************************************************************
Name:
nStopTracking

Return Value:
int - 1 if successful, 0 otherwise

Description:
This routine stops the tracking procedure.
*****************************************************************/
void GetTrackingData2(float q0, float qx, float qy, float qz)
{

	/*fprintf(fp1,"%d \n",TimeMax);
	fprintf(fp2,"%d \n",TimeMax);
	fprintf(fp3,"%d \n",TimeMax);
	fprintf(fp4,"%d \n",TimeMax);
	*/
	int current_time;
	int terminated = 0;
	unsigned long start_time = timeGetTime();
	Switch3 = 0;
	do
	{
		GetSystemTransformData();
		int num = 0;
		for (int j = 0; j<NO_HANDLES; j++)
		{
			if (pCommandHandling->m_dtHandleInformation[j].Xfrms.ulFlags == TRANSFORM_VALID)
			{
				current_time = timeGetTime();
				//pCommandHandling->m_dtHandleInformation[j].Xfrms.translation.x,
				//pCommandHandling->m_dtHandleInformation[j].Xfrms.translation.y,
				//pCommandHandling->m_dtHandleInformation[j].Xfrms.translation.z,
				q0 = -1 * pCommandHandling->m_dtHandleInformation[j].Xfrms.rotation.q0;
				qx = pCommandHandling->m_dtHandleInformation[j].Xfrms.rotation.qx;
				qy = pCommandHandling->m_dtHandleInformation[j].Xfrms.rotation.qy;
				qz = pCommandHandling->m_dtHandleInformation[j].Xfrms.rotation.qz;

			}
			else
			{
				current_time = timeGetTime();
				//for (int k=0; k<7; k++)
				//fprintf(fp1,"MISSING ");
				//fprintf(fp1,"%d \n",current_time);
				//}
			}
		}
		// 終了条件
		current_time = timeGetTime();
		if (abs(current_time - (int)start_time) > TimeMax)
			terminated = 1;
	} while (!terminated);

	//fclose(fp1);
}


/*****************************************************************
StopTracking

Source:
nStopTracking

Return Value:
int - 1 if successful, 0 otherwise

Description:
This routine stops the tracking procedure.
*****************************************************************/
int nStopTracking()
{
	if (pCommandHandling->nStopTracking())
	{
		// set the variable that will stop the thread.
		// set the text on the dialog and put program in proper mode.
		m_bIsTracking = FALSE;
		m_bStopTracking = TRUE;

		return 1;
	}
	return 0;
}

//Polarisの初期化
void SampleViewer::initializedPolaris()
{
	m_bResetHardware = FALSE;
	pCommandHandling = new CCommandHandling;

	cout << "Welcome to Polaris Measurement \n";

	ResetSystem();

	// PolarisとPCの初期化
	m_bSystemInitialized = FALSE;
	while (!m_bSystemInitialized)
		InitializeSystem();

	// Portを開く
	ActivatePorts();
	//cout << "Text Data (0) or Binary Data (1) ? : ";
	//cin >> m_nTrackingMode;
	m_nTrackingMode = 0;

	int sensor_num;
	cout << "Input Sensor Num (1-4) : ";
	cin >> sensor_num;

	/*int th_type;
	cout << "Select LoopMax (0) or TimeMax (1) : ";
	cin >> th_type;
	if (!th_type)
	{
	cout << "Input LoopMax : ";
	cin >> LoopMax;
	if (LoopMax < 0)
	LoopMax = 1000;
	}
	else
	{
	cout << "Input TimeMax[msec] : ";
	cin >> TimeMax;
	if (TimeMax < 0)
	TimeMax = 1000*60*5;
	}
	*/
	// Trackingの開始
	m_bStopTracking = TRUE;
	do
	{
		int answer = 0;
		cout << "Ready ? Yes (1) or No (0) ? : ";
		cin >> answer;
		if (answer)
			m_bStopTracking = FALSE;
	} while (m_bStopTracking);

	if (!StartTracking())
		exit(1);

	/*
	char filename0[] = "port1.dat";

	cout << "Start Tracking \n";
	if (!th_type)
	GetTrackingData(filename0);
	else
	GetTrackingData2(filename0);

	for (int i=0; i<max; i++)
	GetSystemTransformData();
	m_bStopTracking = TRUE;
	*/

	// Trackingの終了
	//nStopTracking(); 
	//CCommandHandling::nStopTracking();
	//cout << "Finish Tracking \n";
	getchar();
}

//クォータニオンの計算
void qtor(double *r, quaternion q)
{
	double xx = q.x * q.x * 2.0;
	double yy = q.y * q.y * 2.0;
	double zz = q.z * q.z * 2.0;
	double xy = q.x * q.y * 2.0;
	double yz = q.y * q.z * 2.0;
	double zx = q.z * q.x * 2.0;
	double xw = q.x * q.w * 2.0;
	double yw = q.y * q.w * 2.0;
	double zw = q.z * q.w * 2.0;
	double r1[16] = { 1.0 - yy - zz, xy + zw, zx - yw, 0.0,
		xy - zw, 1.0 - zz - xx, yz + xw, 0.0,
		zx + yw, yz - xw, 1.0 - xx - yy, 0.0,
		0.0, 0.0, 0.0, 1.0 };
	for (int i = 0; i < 16; i++) {
		r[i] = r1[i];
	}
}
void Search1_(int Deep, int ID) {
	int check = 1;

	for (unsigned int i = 0; i<IB.size(); i++) {
		if (ID == IB[i]) {
			if (IB[i + 1] != -100 && i + 1 <= IB.size()) {
				for (int j = 0; j<TempConnection1.size(); j++) {
					if (IB[i + 1] != TempConnection1[j])
						check = 0;

					else {
						check = 1;
						break;
					}
				}
				if (check == 0) {
					TempConnection1.push_back(IB[i + 1]);
					Deep--;
					if (Deep < 1) {
						return;
					}
					else {
						ID = IB[i + 1];
						Search1_(Deep, ID);
					}
				}
			}
			else if (IB[i - 1] != -100 && i - 1 >= 0) {
				for (int j = 0; j<TempConnection1.size(); j++) {
					if (IB[i - 1] != TempConnection1[j])
						check = 0;

					else {
						check = 1;
						break;
					}
				}
				if (check == 0) {
					TempConnection1.push_back(IB[i - 1]);
					Deep--;
					if (Deep < 1)
						return;
					else {
						ID = IB[i - 1];
						Search1_(Deep, ID);
					}
				}
			}
		}
	}
}
/********************************************
Search1
　: ある頂点IDに接続する頂点を探し、エッジの長さを計算する
 　　vとa、fに初期値0を与える。
   12th Jan 14  S. Sasaki
   ********************************************/
void Search1(int ID) {
	//諸々の初期化
	TempConnection1.clear();
	Edge.clear();
	f_x.clear();
	f_y.clear();
	f_z.clear();
	v_x.clear();
	v_y.clear();
	v_z.clear();
	a_x.clear();
	a_y.clear();
	a_z.clear();

	TempConnection1.push_back(ID);
	Search1_(Depth, ID);

	//TempConnection1
	//cout << "TempConnection数 : " << TempConnection1.size()/2 << "\n";

	//Edge
	for (int i = 0; i<TempConnection1.size() - 1; i++)
	{
		double tempE = 0;
		for (int k = 0; k<3; k++) {
			tempE += (VB[3 * TempConnection1[i] + k - 3] - VB[3 * TempConnection1[i + 1] + k - 3])*(VB[3 * TempConnection1[i] + k - 3] - VB[3 * TempConnection1[i + 1] + k - 3]);
		}
		if (tempE <= 0)
		{
			Edge.push_back(TempConnection1[i]);
			Edge.push_back(TempConnection1[i + 1]);
			Edge.push_back(0);
		}
		else
		{
			Edge.push_back(TempConnection1[i]);
			Edge.push_back(TempConnection1[i + 1]);
			Edge.push_back(sqrt(tempE));
		}

		v_x.push_back(TempConnection1[i]);
		v_x.push_back(TempConnection1[i + 1]);
		v_x.push_back(0);
		v_y.push_back(TempConnection1[i]);
		v_y.push_back(TempConnection1[i + 1]);
		v_y.push_back(0);
		v_z.push_back(TempConnection1[i]);
		v_z.push_back(TempConnection1[i + 1]);
		v_z.push_back(0);
		f_x.push_back(TempConnection1[i]);
		f_x.push_back(TempConnection1[i + 1]);
		f_x.push_back(0);
		f_y.push_back(TempConnection1[i]);
		f_y.push_back(TempConnection1[i + 1]);
		f_y.push_back(0);
		f_z.push_back(TempConnection1[i]);
		f_z.push_back(TempConnection1[i + 1]);
		f_z.push_back(0);
	}
}

/********************************************
Force1
　: 動かす頂点IDに接続する頂点にかかる力f(x,y,z)を計算する

 12th Jan 14  S. Sasaki
 ********************************************/
void Force1(int ID, double fx, double fy, double fz)
{
	//切断面上の点を移動させる
	VB[3 * ID - 3] += fx / M*dt*dt;
	VB[3 * ID - 2] += fy / M*dt*dt;
	VB[3 * ID - 1] += fz / M*dt*dt;
	//cout<<VB[3*ID-3]<<endl<<endl;



	//その他の頂点にかかる力を計算する
	float dx, dy, dz, length;
	//dx.resize(Edge.size())/3,dy.resize(Edge.size())/3,dz.resize(Edge.size())/3,length.resize(Edge.size())/3;

	for (int i = 0; i<Edge.size() / 3; i++)
	{
		if (Edge[3 * i + 1] != ID) {
			int tempID0 = Edge[3 * i];
			int tempID1 = Edge[3 * i + 1];
			//int tempID1=Edgse[3*i];
			//int tempID0=Edge[3*i+1];
			dx = VB[3 * tempID0 - 3] - VB[3 * tempID1 - 3];
			//if(dx==-0)dx=0;
			dy = VB[3 * tempID0 - 2] - VB[3 * tempID1 - 2];
			//if(dy==-0)dy=0;
			dz = VB[3 * tempID0 - 1] - VB[3 * tempID1 - 1];
			//if(z==-0)dz=0;
			length = sqrtf(dx*dx + dy*dy + dz*dz);
			//if(length==-0)length=0;
			//cout << " dx,dy,dz,length " << f_x[3*i+2] << " "<< f_y[3*i+2] <<" "<< f_z[3*i+2] <<" "<< length << "erorr\n";

			f_x[3 * i + 2] = K*(1 - Edge[3 * i + 2] / length)*dx - D*v_x[3 * i + 2];
			f_y[3 * i + 2] = K*(1 - Edge[3 * i + 2] / length)*dy - D*v_y[3 * i + 2];
			f_z[3 * i + 2] = K*(1 - Edge[3 * i + 2] / length)*dz - D*v_z[3 * i + 2];
			//cout<< v_x[3*i]<<endl;
			//cout<< v_x[3*i+1]<<endl;
			//cout<< v_x[3*i+2]<<endl;
			/*if(f_x[3*i+2]!=0)
			cout << f_x[3*i+2] <<endl;
			if(f_y[3*i+2]!=0)
			cout << f_y[3*i+2] <<endl;*/
		}
	}
}
/********************************************
Move1
　: 頂点を動かす

 12th Jan 14  S. Sasaki
 ********************************************/
void Move1(int ID)
{
	//for(int i=0; i<Edge.size()/3; i++){
	//	if(Edge[3*i+1]!=CFPID){
	//		int tempID0=Edge[3*i];
	//		v_x[3*i+2] = f_x[3*i+2]/M*dt;
	//		v_y[3*i+2] = f_y[3*i+2]/M*dt;
	//		v_z[3*i+2] = f_z[3*i+2]/M*dt;

	//		7
	//		VB1[3*tempID0] += v_x[3*i+2]*dt;
	//		VB1[3*tempID0+1] += v_y[3*i+2]*dt;
	//		//VB1[3*tempID0+2] += v_z[3*i+2]*dt;

	//		if(VB1[3*tempID0] < -100)
	//			cout<< i << "Erorr\n";
	//	}
	//}
	for (int i = 0; i<Edge.size() / 3; i++) {
		int fixedend = Edge[Edge.size() - 2];
		if (Edge[3 * i + 1] != ID) {
			int tempID0 = Edge[3 * i + 1];
			v_x[3 * i + 2] = f_x[3 * i + 2] / M*dt;
			v_y[3 * i + 2] = f_y[3 * i + 2] / M*dt;
			v_z[3 * i + 2] = f_z[3 * i + 2] / M*dt;

			if (tempID0 != fixedend) {
				VB[3 * tempID0 - 3] += v_x[3 * i + 2] * dt;
				VB[3 * tempID0 - 2] += v_y[3 * i + 2] * dt;
				VB[3 * tempID0 - 1] += v_z[3 * i + 2] * dt;
			}
			Edge[3 * i + 2] = sqrt((VB[3 * (Edge[3 * i + 1] - 1) + 0] - VB[3 * (Edge[3 * i + 0] - 1) + 0])*(VB[3 * (Edge[3 * i + 1] - 1) + 0] - VB[3 * (Edge[3 * i + 0] - 1) + 0])
				+ (VB[3 * (Edge[3 * i + 1] - 1) + 1] - VB[3 * (Edge[3 * i + 0] - 1) + 1])*(VB[3 * (Edge[3 * i + 1] - 1) + 1] - VB[3 * (Edge[3 * i + 0] - 1) + 1])
				+ (VB[3 * (Edge[3 * i + 1] - 1) + 2] - VB[3 * (Edge[3 * i + 0] - 1) + 2])*(VB[3 * (Edge[3 * i + 1] - 1) + 2] - VB[3 * (Edge[3 * i + 0] - 1) + 2]));

			//if(VB[3*tempID0] < -100)
			//cout<< i << "Erorr\n";
		}
	}
}

/********************************************
Elastic1
　: ばね質点モデルを用いた弾性変形(id1[]を動かす)

 12th Jan 14  S. Sasaki
 ********************************************/
void Elastic1(float MX, float MY, float MZ, int ID1) {

	float x0 = 0, y0 = 0, z0 = 0, d0;
	float x1 = 0, y1 = 0, z1 = 0, d1;
	//float coutVBx,coutVBy,coutVBz;//cout表示用
	//cout << "Elastic1\n";

	//頂点id1と頂点id1との中間地点の変位を求める。
	//coutVBx=VB1[3*ID1], coutVBy=VB1[3*ID1+1], coutVBz=VB1[3*ID1+2];

	x0 = MX - VB[3 * ID1 - 3];
	y0 = MY - VB[3 * ID1 - 2];
	z0 = MZ - VB[3 * ID1 - 1];
	//cout <<MX<<endl;
	d0 = sqrtf(x0*x0 + y0*y0 + z0*z0);
	//CFPID=ID1;
	//cout<<"CFPID = "<< CFPID << " " <<ID1<<"\n";

	//動かす頂点id1に接続している頂点をすべて抜き出す。
	Search1(ID1);

	//頂点IDに接続している頂点にかかる力を計算し、動かす
	float before = FLT_MAX;
	float after = 0;
	for (int l = 0; l<LOOP; l++) {

		Force1(ID1, x0, y0, z0);
		Move1(ID1);

		x1 = MX - VB[3 * ID1 - 3];
		y1 = MY - VB[3 * ID1 - 2];
		z1 = MZ - VB[3 * ID1 - 1];

		d1 = sqrtf(x1*x1 + y1*y1 + z1*z1);
		after = d1;

		if (before - after < 0) {
			VB[3 * ID1 - 3] = MX;
			VB[3 * ID1 - 2] = MY;
			VB[3 * ID1 - 1] = MZ;
			break;
		}

		before = d1;

	}

	//*エッジの大きさをそろえる．（ばねの振動をつりあいの位置に収束させる）※実装に自信なし*//
	/*float cx2,cy2,cz2;
	int cnum1=CFPID;
	int L=0;
	while(L!=5){

	//break;

	++L;

	for(int I=0; I<TempConnection1.size(); I++){

	cnum1=TempConnection1[I];

	if(CFPID!=cnum1){//動かしている頂点でなければ

	cx2=0,cy2=0,cz2=0;
	int countC=0;

	for(int i=0; i<Connection1[CFPID].size(); i++){
	if(Connection1[CFPID][i]==cnum1){
	for(int j=0; j<2; j++){
	cx2+=VB[3*CFPID+0];
	cy2+=VB[3*CFPID+1];
	++countC;
	}
	break;
	}
	}

	for(int j=0; j<Connection1[cnum1].size(); j++){
	if(fabsf(VB[3*cnum1+2]-VB1[3*Connection1[cnum1][j]+2])<0.2){
	cx2+=VB[3*Connection1[cnum1][j]+0];
	cy2+=VB[3*Connection1[cnum1][j]+1];
	//cz2+=VB1[3*Connection1[cnum1][j]+2];
	++countC;
	}
	}

	if(countC>4){
	VB[3*cnum1+0]=cx2/countC;
	VB[3*cnum1+1]=cy2/countC;
	//VB1[3*cnum1+2]=cz2/countC;
	}
	}

	}//loop with I
	}//while
	*/
}
