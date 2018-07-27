/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

/** This file defines Knitro C++ interface constants.
 *  These constants are not standard Knitro constants but are necessary
 *  for Knitro C++ interface
 */

 
/** The following defines the return code to use in evalga when gradient and jacobian
 *  are evaluated in evalfc callback instead of using both evalfc and evalga */
#define KTR_RC_EVALFCGA 10001
