#pragma once

#include <vector>
#include <map>
#include <unordered_set>

#include "Eigen/StdVector"
#include "Eigen/Dense"

#include "util/settings.h"

using namespace Eigen;
using namespace std;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix2f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix3f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix3i);

typedef struct Vertex
{
    Vector3f position;
    bool isActive;    // records whether the corresponding vertex has been deleted or not: if isActive is false then it has been deleted
    Vector3f tangent; // tangent vector
    Vector3f normal;  // normal vector
    Vector3f binormal; // binormal vector
    float strokeWidth;
    // constructor with all fields
    Vertex(Vector3f _position, bool _isActive, Vector3f _tangent, Vector3f _normal, float _strokeWidth)
        : position(_position), isActive(_isActive), tangent(_tangent), normal(_normal), strokeWidth(_strokeWidth){};
    // constructor with all fields except tangent
    Vertex(Vector3f _position, bool _isActive, Vector3f _normal, float _strokeWidth)
        : position(_position), isActive(_isActive), tangent(Vector3f(0, 0, 0)), normal(_normal), strokeWidth(_strokeWidth){};
} Vertex;

class Mesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Mesh();

    void loadFromFile();
    void saveToFile();
    void debugSaveToFile();

    // the main algo
    void getMatches();
    void preprocessLines();
    void getRestrictedMatchingCandidates();
    void meshStripGeneration();

    void cleanUp(); // perform any cleaning up at the end

private:
    // settings
    Settings *settings;

    // common fields
    vector<Vertex *> _vertices;
    vector<vector<int>> _lines;
    vector<Vector3i> _faces;
    // ------- match computation
    unordered_map<int, int> leftMatch; // if the value is -1 then it doesn't have a match
    unordered_map<int, int> rightMatch;
    unordered_map<int, vector<int>> currentMatches; // map from vertex A to a list of vertices that have A as their match (either left or right)
    // ------- restricted matching
    // base vertex index : {
    //          other stroke index : {
    //                          set of vertices on other stroke that matches with base vertex
    unordered_map<int, unordered_map<int, unordered_set<int>>> validVertexVertexMatch;
    // base vertex index : {
    //          other vertex index
    unordered_map<int, unordered_set<int>> leftRestrictedMatchingCandidates;
    unordered_map<int, unordered_set<int>> rightRestrictedMatchingCandidates;
    // ------- mesh strip generation
    unordered_map<int,int> vertsToStrokes;
    // ------- Section 5.4: Manifold consolidation
    unordered_map<std::pair<int, int>, vector<Vector3i>> edgeToTriangles;
    unordered_map<int, vector<Vector3i>> vertexToTriangles;

    // helpers
    vector<vector<int>> parseToPolyline(vector<Vector2i> connections);
    // ------- preprocessing
    void calculateTangentsAndBinormals(const vector<Vector3f> &vertices, const vector<Vector3f> &vertexNormals);
    // ------- match computation
    float vertexVertexScore(Vertex *P, Vertex *Q, bool leftside);
    float persistenceScore(Vertex *Pi, Vertex *Qi, Vertex *Pi_1, Vertex *Qi_1); // Qi is the match of Pi, Qi_1 is the match of Pi_1; Pi and Pi_1 are consecutive vertices
    float computeM(int pi, int qi, int pi_1, int qi_1, bool leftSide);
    vector<vector<int>> getLines();
    vector<int> viterbi(vector<int> S, vector<vector<int>> candidates, bool leftSide); // for testing purposes, moved into public
    // ------- restricted matching
    pair<vector<int>, vector<int>> splitStrokesIntoLeftRight(int baseStrokeIndex);
    bool doTwoVerticesMatch(int pIndex, int qIndex, bool leftside, bool isOnSameStroke, int strokeIndex);
    int calcNumberOfMatches(int baseStrokeIndex, int otherStrokeIndex, bool leftside);
    // ------- mesh strip generation
    std::vector<Vector3i> triangulatePair(int pi,int qi,int pn, int qn);
    // ------- graph generation from triangles
    void makeGraph(std::vector<Vector3i> trianglepatch);

    // ------- Section 5.4: Manifold consolidation
    void populateTriangleMaps();
    vector<vector<Vector3i>> computeUndecidedTriangles();
    bool checkOverlap(int v, int v1, int v2, int v3, int v4);
};
