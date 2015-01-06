#ifndef __LOADSTL_H__
#define __LOADSTL_H__

#include "geom.h"

int loadstl(const char *path, stldata *pstldata);
int writestl(const char *path, stldata stl[]);
int stl_check(stldata *stl);

#endif
