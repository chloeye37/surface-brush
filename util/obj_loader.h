#pragma once
#ifndef OBJ_LOADER_H
#define OBJ_LOADER_H

#include <vector>
#include <string.h>
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

namespace objLoader {

// load vertices and line segments
pair<vector<Vector3f>, vector<Vector2i>> loadFromFile(string fileName);

}

#endif // OBJ_LOADER_H
