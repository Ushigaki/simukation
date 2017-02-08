/*****************************************************************
Name:               StdAfx.h

Description:	Microsoft Developer Studio generated include file
				for standard system include files or project specific 
				include files that are used frequently, but are 
				changed infrequently


All Northern Digital Inc. ("NDI") Media and/or Sample Code and/or
Sample Code Documentation (collectively referred to as "Sample Code")
is licensed and provided "as is" without warranty of any kind.  The
licensee, by use of the Sample Code, warrants to NDI that the Sample
Code is fit for the use and purpose for which the licensee intends to
use the Sample Code. NDI makes no warranties, express or implied, that
the functions contained in the Sample Code will meet the licensee�s
requirements or that the operation of the programs contained therein
will be error free.  This warranty as expressed herein is exclusive
and NDI expressly disclaims any and all express and/or implied, in fact
or in law, warranties, representations, and conditions of every kind
pertaining in any way to the Sample Code licensed and provided by NDI
hereunder, including without limitation, each warranty and/or condition
of quality, merchantability, description, operation, adequacy,
suitability, fitness for particular purpose, title, interference with
use or enjoyment, and/or non infringement, whether express or implied
by statute, common law, usage of trade, course of dealing, custom, or
otherwise.  No NDI dealer, distributor, agent or employee is authorized
to make any modification or addition to this warranty.

In no event shall NDI nor any of its employees be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages,
sundry damages or any damages whatsoever, including, but not limited to,
procurement of substitute goods or services, loss of use, data or profits,
or business interruption, however caused.  In no event shall NDI�s
liability to the licensee exceed the amount paid by the licensee for the S
ample Code or any NDI products that accompany the Sample Code. The said
imitations and exclusions of liability shall apply whether or not any
such damages are construed as arising from a breach of a representation,
warranty, guarantee, covenant, obligation, condition or fundamental term
or on any theory of liability, whether in contract, strict liability, or
tort (including negligence or otherwise) arising in any way out of the
use of the Sample Code even if advised of the possibility of such damage.
In no event shall NDI be liable for any claims, losses, damages, judgments,
costs, awards, expenses or liabilities of any kind whatsoever arising
directly or indirectly from any injury to person or property, arising
from the Sample Code or any use thereof.


Copyright (C) 2002, 2003, Northern Digital Inc. All rights reserved.

 
*****************************************************************/

#if !defined(AFX_STDAFX_H__A5181994_B25E_4379_9465_335BFF080C2C__INCLUDED_)
#define AFX_STDAFX_H__A5181994_B25E_4379_9465_335BFF080C2C__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#define VC_EXTRALEAN		// Exclude rarely-used stuff from Windows headers

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <afxwin.h>         // MFC core and standard components
#include <afxext.h>         // MFC extensions
#include <afxdisp.h>        // MFC Automation classes
#include <afxdtctl.h>		// MFC support for Internet Explorer 4 Common Controls
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>			// MFC support for Windows Common Controls
#endif // _AFX_NO_AFXCMN_SUPPORT
#include <iostream>
#include <MMSystem.h>

#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>
#include <cassert>
//#include <gl/freeglut.h>
#include <math.h>
#include <windows.h>
#include <tchar.h>
#include "INIFileRW.h"
#include "CommandHandling.h"


//�N�H�[�^�j�I���\����
struct quaternion
{
	double w;
	double x;
	double y;
	double z;
};
struct transfer
{
	double x;
	double y;
	double z;
};
//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_STDAFX_H__A5181994_B25E_4379_9465_335BFF080C2C__INCLUDED_)
