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
#ifndef _ONI_SAMPLE_VIEWER_H_
#define _ONI_SAMPLE_VIEWER_H_

#define MAX_DEPTH 10000

/*enum DisplayModes
{
DISPLAY_MODE_OVERLAY,
DISPLAY_MODE_DEPTH,
DISPLAY_MODE_IMAGE
};
*/
#include "Stdafx.h"

class SampleViewer
{
public:
	SampleViewer();
	virtual ~SampleViewer();
	static void INITIALIZEDPOLARIS();
	void GetTrackingData(float &p_q0, float &p_qx, float &p_qy, float &p_qz, float &t_x, float &t_y, float &t_z, float &t_x2, float &t_y2, float &t_z2);
protected:
	virtual void initializedPolaris();

private:
	SampleViewer(const SampleViewer&);
	SampleViewer& operator=(SampleViewer&);

	static SampleViewer* ms_self;
	/*
	float			m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[ONI_MAX_STR];
	unsigned int		m_nTexMapX;
	unsigned int		m_nTexMapY;
	DisplayModes		m_eViewState;
	openni::RGB888Pixel*	m_pTexMap;
	openni::RGB888Pixel*	m_pTexMap_original;*/
	int			m_width;
	int			m_height;
};



#endif // _ONI_SAMPLE_VIEWER_H_
