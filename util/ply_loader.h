#pragma once
#ifndef PLY_LOADER_H
#define PLY_LOADER_H

#include <string.h>
#include <vector>

#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

namespace plyLoader {

// load normals
vector<pair<Vector3f,float>> loadFromFile(string fileName);
vector<pair<Vector3f,float>> alternateloadFromFile(string fileName, vector<vector<int>> lines, vector<Vector3f> vertices);

}

#endif // PLY_LOADER_H
