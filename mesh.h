#pragma once

#include <vector>

#include "Eigen/StdVector"
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix2f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix3f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix3i);

class Mesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void initFromVectors(const vector<Vector3f> &vertices,
                         const vector<vector<int>> &lines);

    void loadFromFile(const string &inObjFilePath, const string &inPlyFilePath);
    void saveToFile(const string &filePath);

private:
    vector<Vector3f> _vertices;
    vector<vector<int>> _lines;
    vector<Vector3f> _vertexNormals;

    // helpers
    vector<vector<int>> parseToPolyline(vector<Vector2i> connections);
    void loadIntoDataStructure();
};
