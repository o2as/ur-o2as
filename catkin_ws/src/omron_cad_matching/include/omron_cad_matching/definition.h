#pragma once

#ifndef MAX_DEPTH
#define MAX_DEPTH 1e+6f
#endif

#ifndef NAN_DEPTH
#define NAN_DEPTH (MAX_DEPTH + 1.0f)
#endif

#ifndef ROUND
#define	ROUND(val) (((val)>=0) ? ((val)+0.5f) : ((val)-0.5f))
#endif

#define SETTING_PARAM_NUM 11
#define OBJCONF_PARAM_NUM 13
