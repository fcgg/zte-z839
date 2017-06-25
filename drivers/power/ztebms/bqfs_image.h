/*****************************************************************************
* Copyright 2010 Texas Instruments Corporation, All Rights Reserved.
* TI makes NO WARRANTY as to software products, which are supplied "AS-IS"
*****************************************************************************/

/*****************************************************************************
*     This code is automatically generated from bqfs/dffs file.              *
*          DO NOT MODIFY THIS FILE DIRECTLY                                  *
*****************************************************************************/
#if defined(CONFIG_BOARD_GIFT)
	#include "bqfs_image_Z936L.h"
#elif defined(CONFIG_BOARD_TSFQJ)//P816T03
	#include "bqfs_image_P816T03.h"
#elif defined(CONFIG_BOARD_XUANTAN)//P816A06
	#include "bqfs_image_P816A06.h"
#elif defined(CONFIG_BOARD_WARP6)//N9518
	#include "bqfs_image_N9518.h"
#elif defined(CONFIG_BOARD_LEO) || defined(CONFIG_BOARD_LOL) //T55&A56
	#include "bqfs_image_P816T55.h"
#elif defined(CONFIG_BOARD_SUNNY) || defined(CONFIG_BOARD_SUNGOD)//P816A13&Z819L
	#include "bqfs_image_P816A13.h"
#elif defined(CONFIG_BOARD_MAX)//N9520+
	#include "bqfs_image_n9520.h"
#elif defined(CONFIG_BOARD_SIF)//P816A15
	#include "bqfs_image_P816A15.h"
#else
	#include "bqfs_image_default.h"
#endif
