#pragma once

#include <vector>
#include <map>
#include <unordered_set>
#include <math.h>
#include <set>

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
    bool isActive;     // records whether the corresponding vertex has been deleted or not: if isActive is false then it has been deleted
    Vector3f tangent;  // tangent vector
    Vector3f normal;   // normal vector
    Vector3f binormal; // binormal vector

    // ------- For gap filling -------------
    Vector3f boundary_tangent; // tangent vector
    Vector3f boundary_binormal;  // binormal vector

    float strokeWidth;
    int componentIndex;
    float componentMatchDistance; // average match distance of the component that this vertex belongs to
    int lineIndex;     // indicates which line (0-indexed) this vertex belongs to
    // constructor with all fields
    Vertex(Vector3f _position, bool _isActive, Vector3f _tangent, Vector3f _normal, float _strokeWidth, int _lineIndex)
        : position(_position), isActive(_isActive), tangent(_tangent), normal(_normal), strokeWidth(_strokeWidth), lineIndex(_lineIndex){};
    // constructor with all fields except tangent and line index
    Vertex(Vector3f _position, bool _isActive, Vector3f _normal, float _strokeWidth)
        : position(_position), isActive(_isActive), tangent(Vector3f(0, 0, 0)), normal(_normal), strokeWidth(_strokeWidth), lineIndex(-1){};
} Vertex;

class Mesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Mesh();

    void loadFromFile();
    void saveToFile();
    void debugSaveToFile();
    void debugUndecidedTrianglesSaveToFile();
    void debugIncompatibleTrianglesSaveToFile();

    // the main algo
    void getMatches();
    void preprocessLines();
    void getRestrictedMatchingCandidates();
    void meshStripGeneration(bool boundary);
    void manifoldConsolidation();
    void cleanFaces();

    void computeUndecidedTriangles(); // temporarily public

    void cleanUp(); // perform any cleaning up at the end

    // ------- Get boundary points
    void computeBoundaries();
    void smoothBoundaries();
    // ------- Generate candidates and then match
    void getBoundaryCandidates();
    void getNewBoundaryTangentsAndBinormals();
    void getBoundaryMatches();



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
    unordered_map<int, int> vertsToStrokes;
    // ------- mesh consolidation
    struct classcomp
    {
        bool operator()(const pair<float, int> &lhs, const pair<float, int> &rhs) const
        {

            return (lhs.first == rhs.first) ? lhs.second > rhs.second : lhs.first > rhs.first;
        }
    };
    // pair< cost, edge encoding >
    // order: largest to smallest cost
    multiset<pair<float, int>, classcomp> edgePriorityQueue;
    unordered_map<int, float> edgeCostMap;
    unordered_map<int, int> unionFindParentMap;
    // --------------------------- makeGraph part starts
    // all edges belonging to output triangles (filled in outputTriangles(), to be used in makeGraph())
    unordered_set<int> outputTrianglesEdges;
    map<std::pair<int, int>, vector<Vector3i>> edgeToTriangles;
    unordered_map<int, vector<Vector3i>> vertexToTriangles;
    unordered_map<int, vector<int>> adjacencyList;
    vector<vector<Vector3i>> undecidedTriangles;
    map<pair<int,int>,vector<Vector3i>> edges_to_triangles;
    // each map in the incompatibleTriangles vector corresponds to one undecidedTriangles's
    // item. so undecidedTriangles.size() == incompatibleTriangles.size();
    vector<unordered_map<int, unordered_set<int>>> incompatibleTriangles;
    // --------------------------- makeGraph part ends

    struct classcomp2
    {
        std::size_t operator()(const pair<Vector3i, Vector3i>& vecPair) const {
          Vector3i vec1 = vecPair.first;
          Vector3i vec2 = vecPair.second;
          std::size_t seed1 = vec1.size();
          std::size_t seed2 = vec2.size();
          for(int i = 0; i < 3; i++) {
              int x = vec1[i];
            x = ((x >> 16) ^ x) * 0x45d9f3b;
            x = ((x >> 16) ^ x) * 0x45d9f3b;
            x = (x >> 16) ^ x;
            seed1 ^= x + 0x9e3779b9 + (seed1 << 6) + (seed1 >> 2);

            int y = vec2[i];
            y = ((y >> 16) ^ y) * 0x45d9f3b;
            y = ((y >> 16) ^ y) * 0x45d9f3b;
            y = (y >> 16) ^ y;
            seed2 ^= y + 0x9e3779b9 + (seed2 << 6) + (seed2 >> 2);
          }
          return utils::elegantPair(min(seed1,seed2), max(seed1,seed2));
        }
    };

    struct classcomp3
    {
        std::size_t operator()(Vector3i const& vec) const {
          std::size_t seed = vec.size();
          for(int i = 0; i < 3; i++) {
              int x = vec[i];
            x = ((x >> 16) ^ x) * 0x45d9f3b;
            x = ((x >> 16) ^ x) * 0x45d9f3b;
            x = (x >> 16) ^ x;
            seed ^= x + 0x9e3779b9 + (seed << 6) + (seed >> 2);
          }
          return seed;
        }
    };

    // { edge encoding :
    //          set< pair< triangle1, triangle2 >> (set of incompatible pairs - order matters!)
    unordered_map<int, unordered_set<pair<Vector3i, Vector3i>, classcomp2>> edgeIncompatibleTrianglesMap;
    // { vertex :
    //          set< pair< triangle1, triangle2 >> (set of incompatible pairs - order matters!)
    unordered_map<int, unordered_set<pair<Vector3i, Vector3i>, classcomp2>> vertexIncompatibleTrianglesMap;
    std::set<int> undecided_vertices;

    // --------------- Section 6: Closing the Gaps -----------------------
    unordered_map<int, float> componentIndex_avgDist;
    void addedgetotrimap(int x, int y, Vector3i thetriangle);
    // ------- Get boundary points
    // If the vector represents a line, the bool is true. If the vector represents a cycle, the bool is false
    // For cycles, the start index is listed at the end as well
    vector<pair<bool,vector<int>>> _boundaries;
    unordered_map<int, unordered_set<int>> boundaryRestrictedMatchingCandidates; // maps index of a boundary vertex to its set of candidates
    unordered_map<int, int> boundaryMatch;
    unordered_map<int, vector<int>> currentBoundaryMatches; // map from vertex A to a list of vertices that have A as their match
    unordered_map<int, vector<int>> boundaryNeighbors; // map from boundary vertex to its one/two neighbors

    // helpers
    vector<vector<int>> parseToPolyline(vector<Vector2i> connections);
    // ------- preprocessing
    void calculateTangentsAndBinormals(const vector<Vector3f> &vertices, const vector<Vector3f> &vertexNormals);
    // ------- match computation
    float vertexVertexScore(Vertex *P, Vertex *Q, bool leftside);
    float persistenceScore(Vertex *Pi, Vertex *Qi, Vertex *Pi_1, Vertex *Qi_1); // Qi is the match of Pi, Qi_1 is the match of Pi_1; Pi and Pi_1 are consecutive vertices
    float computeM(int pi, int qi, int pi_1, int qi_1, bool leftSide);
    vector<vector<int>> getLines();
    vector<int> viterbi(vector<int> S, vector<vector<int>> candidates, bool leftSide, bool boundary); // for testing purposes, moved into public
    // ------- restricted matching
    pair<vector<int>, vector<int>> splitStrokesIntoLeftRight(int baseStrokeIndex);
    bool doTwoVerticesMatch(int pIndex, int qIndex, bool leftside, bool isOnSameStroke, int strokeIndex, bool closeToEnd);
    int calcNumberOfMatches(int baseStrokeIndex, int otherStrokeIndex, bool leftside);

    // ------- Section 5.4: Manifold consolidation

    // ------- Section 6:Boundary Matching
    vector<vector<Vertex*>> getComponents();
    void dfs(unordered_map<int, vector<int>> &adj, vector<bool> &visited, int src, vector<Vertex*> &component, int num_components);
    void getComponentMatchDistance();
    float computeM_boundary(int pi, int qi, int pi_1, int qi_1, bool leftSide);

    std::vector<Vector3i> triangulatePair(int pi, int qi, int pn, int qn);
    std::vector<Vector3i> triangulateBoundaryPair(int pi, int qi, int pn, int qn, int q_stroke_index);
    // ------- mesh consolidation
    bool checkOverlap(int v, int v1, int v2, int v3, int v4);
    bool areVerticesContinuous(Vertex* v1, Vertex* v2);
    unordered_map<int, vector<pair<float, int>>> makeGraph(vector<Vector3i> trianglepatch, unordered_map<int, unordered_set<int>> incompatibles);
    void populateTriangleMaps();
    vector<Vector3i> outputTriangles();
    int encodeEdge(int item1, int item2);
    void createEdgePriorityQueue(unordered_map<int, vector<pair<float, int>>> adjacencies);
    void GAEC(unordered_map<int, vector<pair<float, int>>> adjacencies); // Greedy Additive Edge Contraction
    void KernighanLin();
    vector<int> getNodesUnderSameParentFromUnionFind(int child);
};
