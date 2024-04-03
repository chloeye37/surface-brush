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
    vector<Vector3f> loadFromFile(string fileName);
}

#endif // PLY_LOADER_H
