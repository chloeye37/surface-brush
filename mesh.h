#pragma once

#include <vector>

#include "Eigen/StdVector"
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix2f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix3f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix3i);

struct Vertex{
    Eigen::Vector3f position;
    bool isActive; // records whether the corresponding vertex has been deleted or not: if isActive is false then it has been deleted
    Eigen::Vector3f tangent; // tangent vector


//    int index; // index of the vertex in _vertices

};


class Mesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void initFromVectors(const vector<Vector3f> &vertices,
                         const vector<vector<int>> &lines);

    void loadFromFile(const string &inObjFilePath, const string &inPlyFilePath);
    void saveToFile(const string &outStrokeFilePath, const string &outMeshFilePath);

    void preprocessLines();
    void calculateTangents();

private:
    vector<Vector3f> _vertices;
    vector<vector<int>> _lines;
    vector<Vector3f> _vertexNormals;
    vector<bool> _isActive; // records whether the corresponding vertex has been deleted or not: if isActive is false then it has been deleted
    vector<Vector3f> _faces;
    vector<bool> _isActive; // records whether the corresponding vertex has been deleted or not: if isActive is false then it has been deleted
    vector<Vertex> _m_vertices; // vector of all vertex structs

    // helpers
    vector<vector<int>> parseToPolyline(vector<Vector2i> connections);
};
