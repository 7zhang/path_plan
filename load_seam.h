#ifndef __LOAD_SEAM_H__
#define __LOAD_SEAM_H__

#include "volume.h"
#include "Transform.h"
#include <vector>
using namespace std;

int load_seam(char *path, vector<Vector3D> &p, vector<Vector3D> &n, vector<Vector3D> &t);

#endif