/*------------------------------------------------------------------------------
* ppp_ar.c : ppp ambiguity resolution
*
* options : -DREV_WL_FCB reversed polarity of WL FCB
*
* reference :
*    [1] H.Okumura, C-gengo niyoru saishin algorithm jiten (in Japanese),
*        Software Technology, 1991
*
*          Copyright (C) 2012-2013 by T.TAKASU, All rights reserved.
*
* version : $Revision:$ $Date:$
* history : 2013/03/11 1.0  new
*-----------------------------------------------------------------------------*/
#include "rtklib.h"


/* resolve integer ambiguity for ppp -----------------------------------------*/
extern int pppamb(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav,
				  const double *azel,int *exc)
{
 
    return 0;
}
