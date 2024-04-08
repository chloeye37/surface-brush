#pragma once

#include <vector>

#include "Eigen/StdVector"
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix2f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix3f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix3i);

typedef struct Vertex {
    Vector3f position;
    bool isActive; // records whether the corresponding vertex has been deleted or not: if isActive is false then it has been deleted
    Vector3f tangent; // tangent vector
    Vector3f normal; // normal vector
    // constructor with all fields
    Vertex(Vector3f _position, bool _isActive, Vector3f _tangent, Vector3f _normal)
        : position(_position), isActive(_isActive), tangent(_tangent), normal(_normal) {};
    // constructor with all fields except tangent
    Vertex(Vector3f _position, bool _isActive, Vector3f _normal)
        : position(_position), isActive(_isActive), tangent(Vector3f(0,0,0)), normal(_normal) {};
//    int index; // index of the vertex in _vertices

} Vertex;

class Mesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void loadFromFile(const string &inObjFilePath, const string &inPlyFilePath);
    void saveToFile(const string &outStrokeFilePath, const string &outMeshFilePath);

    void preprocessLines();
    void calculateTangents(const vector<Vector3f> &vertices, const vector<Vector3f> &vertexNormals);

    void cleanUp(); // perform any cleaning up at the end

private:
    float strokewidth = 0.5;
    float sigma = 1.5*(strokewidth+strokewidth)/2;
    vector<Vertex*> _vertices;
    vector<vector<int>> _lines;
    vector<Vector3i> _faces;

    // helpers
    vector<vector<int>> parseToPolyline(vector<Vector2i> connections);
};
