#include "mesh.h"

#include <iostream>
#include <fstream>
#include <array>
#include <deque>

#include <QFileInfo>
#include <QString>
#include <Eigen>

#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"
#include "util/ply_loader.h"
#include "util/obj_loader.h"

#include <fstream>

// -------- PUBLIC STARTS -------------------------------------------------------------------------------
Mesh::Mesh()
{
    this->settings = Settings::getInstance();
}

void Mesh::loadFromFile()
{
    // load obj file
    pair<vector<Vector3f>, vector<Vector2i>> verticesAndLineSegments = objLoader::loadFromFile(this->settings->inObjFile);
    vector<Vector3f> vertices = verticesAndLineSegments.first;
    vector<Vector2i> lineSegments = verticesAndLineSegments.second;
    vector<vector<int>> lines = this->parseToPolyline(lineSegments);

    // load ply file
    vector<pair<Vector3f, float>> normalsAndStrokeWidths = plyLoader::loadFromFile(this->settings->inPlyFile);

    // populate the _vertices vector
    vector<Vertex *> m_vertices;
    for (int i = 0; i < vertices.size(); i++)
    {
        Vertex *vertex = new Vertex(vertices[i], true, normalsAndStrokeWidths[i].first, normalsAndStrokeWidths[i].second);
        // vertex.tangent is set later in calculateTangents()
        // vertex line index is set later in preprocessLines()
        m_vertices.push_back(vertex);
    }

    this->_lines = lines;
    this->_vertices = m_vertices;

    std::vector<Vector3f> normals = std::vector<Vector3f>();
    for (int i = 0; i < normalsAndStrokeWidths.size(); i++)
    {
        normals.push_back(normalsAndStrokeWidths[i].first);
    }

    calculateTangentsAndBinormals(vertices, normals);
}

void Mesh::saveToFile()
{
    ofstream outStrokeFile;
    outStrokeFile.open(this->settings->outStrokeFile);

    ofstream outMeshFile;
    outMeshFile.open(this->settings->outMeshFile);

    // Write vertices
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        Vertex *v = _vertices[i];
        outMeshFile << "v " << v->position[0] << " " << v->position[1] << " " << v->position[2] << endl;
        outStrokeFile << "v " << v->position[0] << " " << v->position[1] << " " << v->position[2] << endl;
    }

    // Write vertex normals
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        Vertex *v = _vertices[i];
        outMeshFile << "vn " << v->normal[0] << " " << v->normal[1] << " " << v->normal[2] << endl;
    }

    // Write faces (MESH ONLY)
    for (size_t i = 0; i < _faces.size(); i++)
    {
        const Vector3i &f = _faces[i];
        outMeshFile << "f " << (f[0] + 1) << " " << (f[1] + 1) << " " << (f[2] + 1) << endl;
    }

    // Write line segments (STROKE ONLY)
    for (size_t i = 0; i < _lines.size(); i++)
    {
        const vector<int> &l = _lines[i];
        for (size_t j = 0; j < l.size() - 1; j++)
        {
            outStrokeFile << "l " << (l[j] + 1) << " " << (l[j + 1] + 1) << endl;
        }
    }

    outStrokeFile.close();
    outMeshFile.close();
}

Vector3i sortvec(Vector3i vec)
{
    int a = vec[0];
    int b = vec[1];
    int c = vec[2];
    if (a <= b && a <= c)
    {
        if (b < c)
        {
            return Vector3i(a, b, c);
        }
        else
        {
            return Vector3i(a, c, b);
        }
    }
    else if (b <= a && b <= c)
    {
        if (a < c)
        {
            return Vector3i(b, a, c);
        }
        else
        {
            return Vector3i(b, c, a);
        }
    }
    else
    {
        if (a < b)
        {
            return Vector3i(c, a, b);
        }
        else
        {
            return Vector3i(c, b, a);
        }
    }
}

int tohash(Vector3i k, int N)
{
    Vector3i sortedvec = sortvec(k);
    return sortedvec[0] * N * N + sortedvec[1] * N + sortedvec[2];
}

int dig(int x, int i, int N)
{
    return ((x % static_cast<int>(pow(N, i + 1))) - (x % static_cast<int>(pow(N, i)))) / pow(N, i);
}

Vector3i fromhash(int x, int N)
{
    return Vector3i(dig(x, 2, N), dig(x, 1, N), dig(x, 0, N));
}

void Mesh::debugSaveToFile()
{
    ofstream outStrokeFile;
    outStrokeFile.open(this->settings->outStrokeFile);
    ofstream outMeshFile;
    outMeshFile.open(this->settings->outMeshFile);

    // Write vertices
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        Vertex *v = _vertices[i];
        outStrokeFile << "v " << v->position[0] << " " << v->position[1] << " " << v->position[2] << endl;
        outMeshFile << "v " << v->position[0] << " " << v->position[1] << " " << v->position[2] << endl;
    }

    // Write strokes
    // Write original line segments
    for (size_t i = 0; i < _lines.size(); i++)
    {
        const vector<int> &l = _lines[i];
        for (size_t j = 0; j < l.size() - 1; j++)
        {
            outStrokeFile << "l " << (l[j] + 1) << " " << (l[j + 1] + 1) << endl;
            outMeshFile << "l " << (l[j] + 1) << " " << (l[j + 1] + 1) << endl;
        }
    }
    // // Write a line segment if there is a match between two points
    //    for (size_t i = 0; i < _vertices.size(); i++)
    //    {
    //        if (leftMatch.contains(i))
    //        {
    //            if (leftMatch.at(i) != -1)
    //            {
    //                outStrokeFile << "l " << i + 1 << " " << leftMatch.at(i) + 1 << endl;
    //                outMeshFile << "l " << i + 1 << " " << leftMatch.at(i) + 1 << endl;
    //            }
    //        }
    //        if (rightMatch.contains(i))
    //        {
    //            if (rightMatch.at(i) != -1)
    //            {
    //                outStrokeFile << "l " << i + 1 << " " << rightMatch.at(i) + 1 << endl;
    //                outMeshFile << "l " << i + 1 << " " << rightMatch.at(i) + 1 << endl;
    //            }
    //        }
    //    }

    // Write boundary matches to StrokeFile
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        if (boundaryMatch.contains(i))
        {
            if (boundaryMatch.at(i) != -1)
            {
                outStrokeFile << "l " << i + 1 << " " << boundaryMatch.at(i) + 1 << endl;
                // outMeshFile << "l " << i + 1 << " " << leftMatch.at(i) + 1 << endl;
            }
        }
    }

    // Look at all face hashes
    //    std::vector<int> facehashes = std::vector<int>();
    //    for (int i = 0; i < _faces.size(); i++)
    //    {
    //        if ((_faces[i][0] == _faces[i][1]) || (_faces[i][1] == _faces[i][2]) || (_faces[i][2] == _faces[i][0]))
    //        {
    //            // std::cout << "Duplicate vertices! " + std::to_string(_faces[i][0]) + ", " + std::to_string(_faces[i][1]) + ", " + std::to_string(_faces[i][2]) << std::endl;
    //        }
    //    }

    // Write faces (MESH ONLY) & boundaries of faces (STROKE ONLY)
    for (size_t i = 0; i < _faces.size(); i++)
    {
        const Vector3i &f = _faces[i];
        outMeshFile << "f " << (f[0] + 1) << " " << (f[1] + 1) << " " << (f[2] + 1) << endl;
        int v1 = f[0] + 1;
        int v2 = f[1] + 1;
        int v3 = f[2] + 1;
        outStrokeFile << "l " << v1 << " " << v2 << endl;
        outStrokeFile << "l " << v2 << " " << v3 << endl;
        outStrokeFile << "l " << v1 << " " << v3 << endl;
    }

    outStrokeFile.close();
    outMeshFile.close();
}

void Mesh::debugUndecidedTrianglesSaveToFile()
{
    ofstream outStrokeFile;
    outStrokeFile.open(this->settings->outStrokeFile);
    ofstream outMeshFile;
    outMeshFile.open(this->settings->outMeshFile);

    // we have: vector<vector<Vector3i>> undecidedTriangles;
    set<int> undecided_vertices;
    vector<int> new_vertex_list;
    vector<Vector3i> undecided_faces;
    unordered_map<int, int> old_to_new; // map of vertex indices from old (in _vertices) to new (in undecided_vertices)
    unordered_map<int, int> new_to_old;
    for (auto &triangle_list : undecidedTriangles)
    {
        for (auto triangle : triangle_list)
        {
            undecided_vertices.insert(triangle[0]);
            undecided_vertices.insert(triangle[1]);
            undecided_vertices.insert(triangle[2]);
        }
    }
    int new_index = 0;
    for (auto vertex : undecided_vertices)
    {
        new_vertex_list.push_back(new_index);
        old_to_new.insert({vertex, new_index});
        new_to_old.insert({new_index, vertex});
        new_index++;
    }
    for (auto new_vertex : new_vertex_list)
    {
        outMeshFile << "v " << _vertices[new_to_old.at(new_vertex)]->position[0] << " " << _vertices[new_to_old.at(new_vertex)]->position[1] << " " << _vertices[new_to_old.at(new_vertex)]->position[2] << endl;
        outStrokeFile << "v " << _vertices[new_to_old.at(new_vertex)]->position[0] << " " << _vertices[new_to_old.at(new_vertex)]->position[1] << " " << _vertices[new_to_old.at(new_vertex)]->position[2] << endl;
    }

    for (auto &triangle_list : undecidedTriangles)
    {
        for (auto triangle : triangle_list)
        {

            // then write lines
            outMeshFile << "l " << (old_to_new.at(triangle[0]) + 1) << " " << (old_to_new.at(triangle[1]) + 1) << endl;
            outMeshFile << "l " << (old_to_new.at(triangle[1]) + 1) << " " << (old_to_new.at(triangle[2]) + 1) << endl;
            outMeshFile << "l " << (old_to_new.at(triangle[2]) + 1) << " " << (old_to_new.at(triangle[0]) + 1) << endl;
            // then write faces for mesh file
            outMeshFile << "f " << (old_to_new.at(triangle[0]) + 1) << " " << (old_to_new.at(triangle[1]) + 1) << " " << (old_to_new.at(triangle[2]) + 1) << endl;

            outStrokeFile << "l " << (old_to_new.at(triangle[0]) + 1) << " " << (old_to_new.at(triangle[1]) + 1) << endl;
            outStrokeFile << "l " << (old_to_new.at(triangle[1]) + 1) << " " << (old_to_new.at(triangle[2]) + 1) << endl;
            outStrokeFile << "l " << (old_to_new.at(triangle[2]) + 1) << " " << (old_to_new.at(triangle[0]) + 1) << endl;
        }
    }

    outStrokeFile.close();
    outMeshFile.close();
}
// Preprocess the lines: check if the strokes have an abrupt direction change (angle of 45◦ or less between consecutive tangents)
// within 15% of overall stroke length from either end and remove the offending end-sections.
void Mesh::preprocessLines()
{
    vector<vector<int>> new_lines;
    int itr = 0;
    // process each line at a time
    for (auto &line : _lines)
    {
        // first calculate line length -- sum of all segment lengths
        float total_length = 0;
        for (int i = 1; i < line.size(); i++)
        {
            // distance between vertex i and vertex i-1
            total_length += (this->_vertices[line[i]]->position - this->_vertices[line[i - 1]]->position).norm();
        }

        float length_covered = 0;
        // forward direction: start to end until length_covered > 0.15 * total_length
        // calculate the tangents along the way and test if they differ by more than 45 degrees
        Vector3f prev_tangent = this->_vertices[line[1]]->position - this->_vertices[line[0]]->position; // tangent of the first vertex is just the line segment direction
        int cur_vert = 0;
        int next_vert = 1;
        int cut_pos_forward = 0; // the vertex at which we should cut the line (remove every vertex before this cut_pos)
        while (length_covered < 0.15 * total_length)
        {
            Vector3f curA = this->_vertices[line[cur_vert]]->position;
            Vector3f curB = this->_vertices[line[next_vert]]->position;
            Vector3f curC = this->_vertices[line[next_vert + 1]]->position;
            Vector3f AC = curC - curA;
            Vector3f B_normal = this->_vertices[line[next_vert]]->normal;
            Vector3f AC_parallel = AC.dot(B_normal) / (B_normal.norm() * B_normal.norm()) * B_normal; // AC projected onto the direction of normal

            Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
            if (acos(cur_tangent.dot(prev_tangent) / (cur_tangent.norm() * prev_tangent.norm())) > M_PI / 4.0)
            {
                // remove everything before vertex B (starting from A)
                cut_pos_forward = next_vert;
            }
            // move on to the next segment
            prev_tangent = cur_tangent;
            length_covered += (curB - curA).norm();
            cur_vert++;
            next_vert++;
        }

        // backward direction: end to start
        int n = line.size();
        length_covered = 0;
        prev_tangent = this->_vertices[line[n - 2]]->position - this->_vertices[line[n - 1]]->position; // tangent of the first vertex is just the line segment direction
        cur_vert = n - 1;
        next_vert = n - 2;
        int cut_pos_backward = n - 1; // the vertex at which we should cut the line (remove every vertex after this cut_pos)
        while (length_covered < 0.15 * total_length)
        {
            Vector3f curA = this->_vertices[line[cur_vert]]->position;
            Vector3f curB = this->_vertices[line[next_vert]]->position;
            Vector3f curC = this->_vertices[line[next_vert - 1]]->position;
            Vector3f AC = curC - curA;
            Vector3f B_normal = this->_vertices[line[next_vert]]->normal;
            Vector3f AC_parallel = AC.dot(B_normal) / (B_normal.norm() * B_normal.norm()) * B_normal; // AC projected onto the direction of normal

            Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
            if (acos(cur_tangent.dot(prev_tangent) / (cur_tangent.norm() * prev_tangent.norm())) > M_PI / 4.0)
            {
                // remove everything before vertex B (starting from A)
                cut_pos_backward = next_vert;
            }
            // move on to the next segment
            prev_tangent = cur_tangent;
            length_covered += (curB - curA).norm();
            cur_vert--;
            next_vert--;
        }

        // remove everything before cut_pos_forward and after cut_pos_backward
        // need to modify: _vertices (vector<Vector3f>), _lines (vector<vector<int>>), _vertexNormals (vector<Vector3f>)
        vector<int> new_line;
        for (int i = cut_pos_forward; i <= cut_pos_backward; i++)
        {
            new_line.push_back(line[i]);
        }
        new_lines.push_back(new_line);

        // logical removal of vertices
        for (int i = 0; i < cut_pos_forward; i++)
        {
            _vertices[line[i]]->isActive = false;
        }

        for (int i = n - 1; i > cut_pos_backward; i--)
        {
            _vertices[line[i]]->isActive = false;
        }
        itr++;
    }
    _lines = new_lines;

    // update the _vertices and _lines (using the correct indices)
    // - update _vertices
    unordered_map<int, int> index_map; // old index -> new index
    int active_number = 0;
    vector<Vertex *> new_vertices;
    for (int i = 0; i < _vertices.size(); i++)
    {
        if (_vertices[i]->isActive)
        {
            index_map[i] = active_number;
            active_number++;
            new_vertices.push_back(this->_vertices[i]);
        }
        else
        {
            delete this->_vertices[i];
        }
    }
    this->_vertices = new_vertices;

    // - remap the vertex indices in _lines
    vector<vector<int>> final_lines;
    for (auto &line : _lines)
    {
        vector<int> final_line;
        for (int vertex : line)
        {
            final_line.push_back(index_map[vertex]);
        }
        final_lines.push_back(final_line);
    }
    this->_lines = final_lines;

    // NEW NEW NEW NEW
    // also, add lineIndex here for vertices
    for (int i = 0; i < this->_lines.size(); i++)
    {
        for (int j = 0; j < (this->_lines[i]).size(); j++)
        {
            int vertexIndex = this->_lines[i][j];
            vertsToStrokes.emplace(vertexIndex, i);
            this->_vertices[vertexIndex]->lineIndex = i;
        }
    }
}

void Mesh::getRestrictedMatchingCandidates()
{
    assert(!this->_lines.empty());

    for (int i = 0; i < this->_lines.size(); i++)
    {
        vector<int> stroke = this->_lines[i];
        // split rest of strokes into left & right strokes
        pair<vector<int>, vector<int>> split = this->splitStrokesIntoLeftRight(i);
        vector<int> leftStrokeIndices = split.first;
        vector<int> rightStrokeIndices = split.second;

        int highestLeftMatches = 0;
        int leftDominantStrokeIndex = -1;
        for (int leftStrokeIndex : leftStrokeIndices)
        {
            // get highest matches stroke
            int matches = this->calcNumberOfMatches(i, leftStrokeIndex, true);
            if (matches > highestLeftMatches)
            {
                leftDominantStrokeIndex = leftStrokeIndex;
                highestLeftMatches = matches;
            }
        }
        // if leftDominantStrokeIndex == -1, then no left dominant stroke!

        int highestRightMatches = 0;
        int rightDominantStrokeIndex = -1;
        for (int rightStrokeIndex : rightStrokeIndices)
        {
            // get highest matches stroke
            int matches = this->calcNumberOfMatches(i, rightStrokeIndex, false);
            if (matches > highestRightMatches)
            {
                rightDominantStrokeIndex = rightStrokeIndex;
                highestRightMatches = matches;
            }
        }
        // if rightDominantStrokeIndex == -1, then no right dominant stroke!

        // get restricted matching candidates
        for (int strokeVertexIndex = 0; strokeVertexIndex < stroke.size(); strokeVertexIndex++)
        {
            int vertexIndex = stroke[strokeVertexIndex]; // index of vertex in vector of vertices
            if (!this->validVertexVertexMatch.contains(vertexIndex))
            {
                this->validVertexVertexMatch[vertexIndex] = unordered_map<int, unordered_set<int>>();
            }
            unordered_map<int, unordered_set<int>> dominantStrokeMaps = this->validVertexVertexMatch.at(vertexIndex);

            unordered_set<int> vertexLeftRestrictedMatchingCandidates;
            if (leftDominantStrokeIndex != -1)
            {
                if (!dominantStrokeMaps.contains(leftDominantStrokeIndex))
                {
                    dominantStrokeMaps[leftDominantStrokeIndex] = unordered_set<int>();
                }
                unordered_set<int> leftDominantStrokeMatches = dominantStrokeMaps.at(leftDominantStrokeIndex);
                vertexLeftRestrictedMatchingCandidates.merge(leftDominantStrokeMatches);
            }
            this->leftRestrictedMatchingCandidates[vertexIndex] = vertexLeftRestrictedMatchingCandidates;

            unordered_set<int> vertexRightRestrictedMatchingCandidates;
            if (rightDominantStrokeIndex != -1)
            {
                if (!dominantStrokeMaps.contains(rightDominantStrokeIndex))
                {
                    dominantStrokeMaps[rightDominantStrokeIndex] = unordered_set<int>();
                }
                unordered_set<int> rightDominantStrokeMatches = dominantStrokeMaps.at(rightDominantStrokeIndex);
                vertexRightRestrictedMatchingCandidates.merge(rightDominantStrokeMatches);
            }
            this->rightRestrictedMatchingCandidates[vertexIndex] = vertexRightRestrictedMatchingCandidates;
        }

        // also consider vertices in the same stroke
        for (int strokeVertexIndex = 0; strokeVertexIndex < stroke.size(); strokeVertexIndex++)
        {
            int vertexIndex = stroke[strokeVertexIndex];
            for (int strokeVertexIndex2 = 0; strokeVertexIndex2 < stroke.size(); strokeVertexIndex2++)
            {
                int vertexIndex2 = stroke[strokeVertexIndex2];

                if (vertexIndex == vertexIndex2)
                    continue;

                bool isEitherVertexCloseToEnd = (vertexIndex > stroke.size() - this->settings->noOfNearEndVerticesToConsider || vertexIndex < this->settings->noOfNearEndVerticesToConsider || vertexIndex2 > stroke.size() - this->settings->noOfNearEndVerticesToConsider || vertexIndex2 < this->settings->noOfNearEndVerticesToConsider);

                if (this->doTwoVerticesMatch(vertexIndex, vertexIndex2, true, true, i, isEitherVertexCloseToEnd))
                {
                    if (!this->leftRestrictedMatchingCandidates.contains(vertexIndex))
                    {
                        this->leftRestrictedMatchingCandidates[vertexIndex] = unordered_set<int>();
                    }
                    unordered_set<int> leftMatches = this->leftRestrictedMatchingCandidates.at(vertexIndex);
                    leftMatches.insert(vertexIndex2);
                    this->leftRestrictedMatchingCandidates[vertexIndex] = leftMatches;
                }
                else if (this->doTwoVerticesMatch(vertexIndex, vertexIndex2, false, true, i, isEitherVertexCloseToEnd))
                {
                    if (!this->rightRestrictedMatchingCandidates.contains(vertexIndex))
                    {
                        this->rightRestrictedMatchingCandidates[vertexIndex] = unordered_set<int>();
                    }
                    unordered_set<int> rightMatches = this->rightRestrictedMatchingCandidates.at(vertexIndex);
                    rightMatches.insert(vertexIndex2);
                    this->rightRestrictedMatchingCandidates[vertexIndex] = rightMatches;
                }
            }
        }
    }
}

pair<int,int> sortpair(int x, int y){
    if(x < y){
        return make_pair(x,y);
    }
    return make_pair(y,x);
}

void Mesh::addedgetotrimap(int x, int y, Vector3i thetriangle){
    pair<int,int> thepair = sortpair(x,y);
    if(edges_to_triangles.contains(thepair)){
        edges_to_triangles.at(thepair).push_back(thetriangle);
    }
    else{
        vector<Vector3i> newvec;
        newvec.push_back(thetriangle);
        edges_to_triangles.emplace(thepair, newvec);
    }
}

void Mesh::meshStripGeneration(bool boundary) {
    unordered_set<int> trihashes;
    vector<vector<int>> strokes;
    unordered_map<int, int> boundaryVertsToStrokes;

    if (boundary) {
        // vector<pair<bool,vector<int>>> _boundaries; -> turn into a vector<vector<int>>
        for (int i = 0; i < _boundaries.size(); i++) {
            auto pair = _boundaries[i];
            strokes.push_back(pair.second);
            for (int boundary_vertex : pair.second) {
                boundaryVertsToStrokes.insert({boundary_vertex, i});
            }
        }
    } else {
        strokes = _lines;
    }

    for (int i = 0; i < strokes.size(); i++)
    {
        // Go through the stroke
        std::vector<int> currstroke = strokes[i];
        for (int j = 0; j < currstroke.size() - 1; j++)
        {
            // Go through each vertex
            int pi = currstroke[j];
            if (!boundary) {
                // Check if the next vertex even exists
                if (leftMatch.contains(pi) && leftMatch.contains(pi + 1))
                {
                    int qi = leftMatch.at(pi);
                    int qn = leftMatch.at(pi + 1);
                    if ((qi >= 0) && (qn >= 0) && (vertsToStrokes.at(qi) == vertsToStrokes.at(qn)))
                    {
                        // Both this and the next vertex have matches.
                        std::vector<Vector3i> outtriangles = triangulatePair(pi, qi, pi + 1, qn);
                        for (int k = 0; k < outtriangles.size(); k++)
                        {
                            // Check if the triangle exists already
                            int trihash = tohash(outtriangles[k], _vertices.size());
                            if (!trihashes.contains(trihash))
                            {
                                _faces.push_back(outtriangles[k]);
                                trihashes.insert(trihash);
                                addedgetotrimap(outtriangles[k][0],outtriangles[k][1], outtriangles[k]);
                                addedgetotrimap(outtriangles[k][1],outtriangles[k][2], outtriangles[k]);
                                addedgetotrimap(outtriangles[k][2],outtriangles[k][0], outtriangles[k]);
                            }
                        }
                    }
                }
                if (rightMatch.contains(pi) && rightMatch.contains(pi + 1))
                {
                    int qi = rightMatch.at(pi);
                    int qn = rightMatch.at(pi + 1);
                    if ((qi >= 0) && (qn >= 0) && (vertsToStrokes.at(qi) == vertsToStrokes.at(qn)))
                    {
                        // Both this and the next vertex have matches.
                        std::vector<Vector3i> outtriangles = triangulatePair(pi, qi, pi + 1, qn);
                        for (int k = 0; k < outtriangles.size(); k++)
                        {
                            // Check if the triangle exists already
                            int trihash = tohash(outtriangles[k], _vertices.size());
                            if (!trihashes.contains(trihash))
                            {
                                _faces.push_back(outtriangles[k]);
                                trihashes.insert(trihash);
                                addedgetotrimap(outtriangles[k][0],outtriangles[k][1], outtriangles[k]);
                                addedgetotrimap(outtriangles[k][1],outtriangles[k][2], outtriangles[k]);
                                addedgetotrimap(outtriangles[k][2],outtriangles[k][0], outtriangles[k]);
                            }
                        }
                    }
                }
            } else {
                // Check if the next vertex even exists
                if (boundaryMatch.contains(pi) && boundaryMatch.contains(currstroke[j+1]))
                {
                    int qi = boundaryMatch.at(pi);
                    int qn = boundaryMatch.at(currstroke[j+1]);
                    if ((qi >= 0) && (qn >= 0) && (boundaryVertsToStrokes.at(qi) == boundaryVertsToStrokes.at(qn)))
                    {
                        // Both this and the next vertex have matches.
                        std::vector<Vector3i> outtriangles = triangulatePair(pi, qi, currstroke[j+1], qn);
                        for (int k = 0; k < outtriangles.size(); k++)
                        {
                            // Check if the triangle exists already
                            int trihash = tohash(outtriangles[k], _vertices.size());
                            if (!trihashes.contains(trihash))
                            {
                                _faces.push_back(outtriangles[k]);
                                trihashes.insert(trihash);
                                // addedgetotrimap(outtriangles[k][0],outtriangles[k][1], outtriangles[k]);
                                // addedgetotrimap(outtriangles[k][1],outtriangles[k][2], outtriangles[k]);
                                // addedgetotrimap(outtriangles[k][2],outtriangles[k][0], outtriangles[k]);
                            }
                        }
                    }
                }
            }

        }
    }
}

void Mesh::manifoldConsolidation()
{
    this->computeUndecidedTriangles();
    vector<Vector3i> outputTriangles = this->outputTriangles();
    // NOTE: LINE BELOW IS DEBUG ONLY
    //    outputTriangles.clear();

    for (int i = 0; i < this->undecidedTriangles.size(); i++)
    {
        vector<Vector3i> trianglePatch = this->undecidedTriangles[i];
        unordered_map<int, unordered_set<int>> curPatchIncompatibleTriangles = this->incompatibleTriangles[i];
        int rootNodeIndex = trianglePatch.size();
        unordered_map<int, vector<pair<float, int>>> adjacencies = this->makeGraph(trianglePatch, curPatchIncompatibleTriangles);
        this->GAEC(adjacencies);
        // get all nodes that are connected to root node
        vector<int> outputChildren = this->getNodesUnderSameParentFromUnionFind(rootNodeIndex);
        // triangle indices in trianglePatch should be the same as node indices in adjacencies
        for (int outputChild : outputChildren)
        {
            outputTriangles.push_back(trianglePatch[outputChild]);
        }

        // clear, prep for next graph
        this->edgePriorityQueue.clear();
        this->edgeCostMap.clear();
        this->unionFindParentMap.clear();
    }

    this->_faces = outputTriangles;
}

void Mesh::cleanUp()
{
    for (int i = 0; i < this->_vertices.size(); i++)
    {
        delete this->_vertices[i];
    }
}

vector<vector<int>> Mesh::getLines()
{
    return _lines;
}

// -------- PUBLIC ENDS -------------------------------------------------------------------------------

// -------- PRIVATE STARTS -------------------------------------------------------------------------------

vector<vector<int>> Mesh::parseToPolyline(vector<Vector2i> connections)
{
    // TODO: Sort connections first by the first index!
    vector<Vector2i> sortedconns = connections;

    struct sort_pred
    {
        bool operator()(const Vector2i &left, const Vector2i &right)
        {
            return left[0] < right[0];
        }
    };

    std::sort(sortedconns.begin(), sortedconns.end(), sort_pred());

    std::vector<std::vector<int>> polylines = std::vector<std::vector<int>>();
    int index = 0;
    int strokedex = 0;
    while (index < sortedconns.size())
    {
        // Add vertices to the vertsToStrokes map! (NOTE: this is actually done later in preprocessLines)
        vector<int> currentpoly = std::vector<int>();
        currentpoly.push_back(sortedconns[index][0]);
        currentpoly.push_back(sortedconns[index][1]);

        index = index + 1;
        while (sortedconns[index][0] == currentpoly[currentpoly.size() - 1])
        {
            currentpoly.push_back(sortedconns[index][1]);
            index = index + 1;
        }
        polylines.push_back(currentpoly);
        strokedex = strokedex + 1;
    }

    return polylines;
}

// --------------------- Section 5.4 ------------------------
// --------------------- Get undecided triangle clusters ------------------------
// A function to create a map that associates each vertex with its list of triangles, and a map that associates every edge with its triangles
// Also populates an adjacency list for section 6
void Mesh::populateTriangleMaps() {
    for (Vector3i &triangle : _faces) {
        // first populate unordered_map<std::pair<int, int>, Vector3i> edgeToTriangles
        int v1 = triangle[0];
        int v2 = triangle[1];
        int v3 = triangle[2];
        auto pair12 = v1 < v2 ? std::make_pair(v1, v2) : std::make_pair(v2, v1); // edge ordered by smaller vertex -- larger vertex
        if (edgeToTriangles.contains(pair12))
        {
            edgeToTriangles.at(pair12).push_back(triangle);
        }
        else
        {
            edgeToTriangles.insert({pair12, {triangle}});
        }
        auto pair13 = v1 < v3 ? std::make_pair(v1, v3) : std::make_pair(v3, v1); // edge ordered by smaller vertex -- larger vertex
        if (edgeToTriangles.contains(pair13))
        {
            edgeToTriangles.at(pair13).push_back(triangle);
        }
        else
        {
            edgeToTriangles.insert({pair13, {triangle}});
        }
        auto pair23 = v3 < v2 ? std::make_pair(v3, v2) : std::make_pair(v2, v3); // edge ordered by smaller vertex -- larger vertex
        if (edgeToTriangles.contains(pair23))
        {
            edgeToTriangles.at(pair23).push_back(triangle);
        }
        else
        {
            edgeToTriangles.insert({pair23, {triangle}});
        }

        // then populate unordered_map<int, Vector3i> vertexToTriangles
        if (vertexToTriangles.contains(v1))
        {
            vertexToTriangles.at(v1).push_back(triangle);
        }
        else
        {
            vertexToTriangles.insert({v1, {triangle}});
        }
        if (vertexToTriangles.contains(v2))
        {
            vertexToTriangles.at(v2).push_back(triangle);
        }
        else
        {
            vertexToTriangles.insert({v2, {triangle}});
        }
        if (vertexToTriangles.contains(v3))
        {
            vertexToTriangles.at(v3).push_back(triangle);
        }
        else
        {
            vertexToTriangles.insert({v3, {triangle}});
        }

        // finally populate unordered_map<int, vector<int>> adjacencyList
        // v1 should have v2 and v3 in its adjacency list, and so on
        if (adjacencyList.contains(v1)) {
            // if v2 is not already inside v1's adjacency list, add v2
            if (std::find(adjacencyList.at(v1).begin(), adjacencyList.at(v1).end(), v2) == adjacencyList.at(v1).end()) adjacencyList.at(v1).push_back(v2);
            // if v3 is not already inside v1's adjacency list, add v3
            if (std::find(adjacencyList.at(v1).begin(), adjacencyList.at(v1).end(), v3) == adjacencyList.at(v1).end()) adjacencyList.at(v1).push_back(v3);
        }else {
            adjacencyList.insert({v1, {v2, v3}});
        }
        if (adjacencyList.contains(v2)) {
            // if v1 is not already inside v2's adjacency list, add v1
            if (std::find(adjacencyList.at(v2).begin(), adjacencyList.at(v2).end(), v1) == adjacencyList.at(v2).end()) adjacencyList.at(v2).push_back(v1);
            // if v3 is not already inside v2's adjacency list, add v3
            if (std::find(adjacencyList.at(v2).begin(), adjacencyList.at(v2).end(), v3) == adjacencyList.at(v2).end()) adjacencyList.at(v2).push_back(v3);
        }else {
            adjacencyList.insert({v2, {v1, v3}});
        }
        if (adjacencyList.contains(v3)) {
            // if v2 is not already inside v3's adjacency list, add v2
            if (std::find(adjacencyList.at(v3).begin(), adjacencyList.at(v3).end(), v2) == adjacencyList.at(v3).end()) adjacencyList.at(v3).push_back(v2);
            // if v1 is not already inside v3's adjacency list, add v1
            if (std::find(adjacencyList.at(v3).begin(), adjacencyList.at(v3).end(), v1) == adjacencyList.at(v3).end()) adjacencyList.at(v3).push_back(v1);
        }else {
            adjacencyList.insert({v3, {v2, v1}});
        }

    }
    // also use edges from _lines: neighboring vertices are connected: vector<vector<int>> _lines;
    for (auto &line : _lines) {
        for (int i = 0; i < line.size(); i++) {
            if (adjacencyList.contains(line[i])) {
                if (i > 0) {
                    // if previous vertex is not already inside i's adjacency list, add previous vertex
                    if (std::find(adjacencyList.at(line[i]).begin(), adjacencyList.at(line[i]).end(), line[i-1]) == adjacencyList.at(line[i]).end()) adjacencyList.at(line[i]).push_back(line[i-1]);
                }
                if (i < line.size() - 1) {
                    // if next vertex is not already inside i's adjacency list, add next vertex
                    if (std::find(adjacencyList.at(line[i]).begin(), adjacencyList.at(line[i]).end(), line[i+1]) == adjacencyList.at(line[i]).end()) adjacencyList.at(line[i]).push_back(line[i+1]);
                }
            }else {
                if (i == 0) adjacencyList.insert({line[i], {line[i+1]}});
                else if (i == line.size()-1) adjacencyList.insert({line[i], {line[i-11]}});
                else adjacencyList.insert({line[i], {line[i-1], line[i+1]}});
            }
        }
    }
}

// We have all the triangles (triangles: vector<int> = {v1, v2, v3}), need to identify the incompatible ones
// Populates vector<vector<Vector3i>> undecidedTriangles
void Mesh::computeUndecidedTriangles()
{
    unordered_set<pair<Vector3i, Vector3i>, classcomp2> emptySet; // reusable empty set

    populateTriangleMaps();
    vector<vector<Vector3i>> res;
    std::set<std::pair<int, int>> undecided_edges;
    // std::set<int> undecided_vertices;
    // Case (1) and (3): look for triangles that share one edge
    for (auto i = edgeToTriangles.begin(); i != edgeToTriangles.end(); i++)
    {
        auto cur_edge = i->first;
        auto [v1, v2] = i->first;
        if (abs(v1 - v2) > 1)
            continue; // only consider the case where shared edge is on the same stroke
        vector<Vector3i> triangles = i->second;
        // loop through all triangles and check them pairwise
        for (Vector3i &t1 : triangles) {
            for (Vector3i &t2 : triangles) {
                int v3, v4;
                // get v3 and v4
                if (t1[0] != v1 && t1[0] != v2)
                    v3 = t1[0];
                else if (t1[1] != v1 && t1[1] != v2)
                    v3 = t1[1];
                else
                    v3 = t1[2];
                if (t2[0] != v1 && t2[0] != v2)
                    v4 = t2[0];
                else if (t2[1] != v1 && t2[1] != v2)
                    v4 = t2[1];
                else
                    v4 = t2[2];
                if (v3 == v4)
                    continue; // same triangle...
                // Case (1): check if v3 and v4 are on different sides of the stroke
                Vector3f edge = _vertices[v1]->position - _vertices[v2]->position;
                Vector3f t1edge = _vertices[v3]->position - _vertices[v2]->position;
                Vector3f t2edge = _vertices[v4]->position - _vertices[v2]->position;
                if ((t1edge.dot(_vertices[v2]->binormal)) * (t2edge.dot(_vertices[v2]->binormal)) > 0)
                { // same side
                    // mark this edge
                    undecided_edges.insert(cur_edge);

                    // add incompatible triangles to map
                    int edgeEncoding = this->encodeEdge(cur_edge.first, cur_edge.second);
                    if (!this->edgeIncompatibleTrianglesMap.contains(edgeEncoding))
                    {
                        this->edgeIncompatibleTrianglesMap[edgeEncoding] = emptySet;
                    }
                    unordered_set<pair<Vector3i, Vector3i>, classcomp2> incompatibleTriangleSet = this->edgeIncompatibleTrianglesMap.at(edgeEncoding);
                    if (!incompatibleTriangleSet.contains(make_pair(t1, t2)) && !incompatibleTriangleSet.contains(make_pair(t2, t1)))
                    {
                        incompatibleTriangleSet.insert(make_pair(t1, t2));
                        this->edgeIncompatibleTrianglesMap[edgeEncoding] = incompatibleTriangleSet;
                    }
                }
                // Case (3): check if the dihedral angle is less than 45 degrees
                // https://math.stackexchange.com/questions/47059/how-do-i-calculate-a-dihedral-angle-given-cartesian-coordinates
                Vector3f b2 = edge;
                Vector3f b1 = -t1edge;
                Vector3f b3 = t2edge;
                Vector3f n1 = b1.cross(b2).normalized();
                Vector3f n2 = b2.cross(b3).normalized();
                float x = n1.dot(n2);
                float y = (n1.cross(b2.normalized())).dot(n2);
                if (abs(atan2(y, x) * 180 / M_PI) < 45)
                { // less than 45 degrees
                    undecided_edges.insert(cur_edge);

                    // add incompatible triangles to map
                    int edgeEncoding = this->encodeEdge(cur_edge.first, cur_edge.second);
                    if (!this->edgeIncompatibleTrianglesMap.contains(edgeEncoding))
                    {
                        this->edgeIncompatibleTrianglesMap[edgeEncoding] = emptySet;
                    }
                    unordered_set<pair<Vector3i, Vector3i>, classcomp2> incompatibleTriangleSet = this->edgeIncompatibleTrianglesMap.at(edgeEncoding);
                    if (!incompatibleTriangleSet.contains(make_pair(t1, t2)) && !incompatibleTriangleSet.contains(make_pair(t2, t1)))
                    {
                        incompatibleTriangleSet.insert(make_pair(t1, t2));
                        this->edgeIncompatibleTrianglesMap[edgeEncoding] = incompatibleTriangleSet;
                    }
                }
            }
        }
    }
    // Case (2): look for triangles that share one vertex
    for (auto i = vertexToTriangles.begin(); i != vertexToTriangles.end(); i++)
    {
        int v = i->first;
        vector<Vector3i> triangles = i->second;
        // loop through all triangles and check them pairwise
        for (Vector3i &t1 : triangles) {
            for (Vector3i &t2 : triangles) {
                // first check if they are on the same side of the stroke
                // v1 and v2 belong to t1, v3 and v4 belong to t2
                int v1, v2, v3, v4;
                if (t1[0] == v)
                {
                    v1 = t1[1];
                    v2 = t1[2];
                }
                else if (t1[1] == v)
                {
                    v1 = t1[0];
                    v2 = t1[2];
                }
                else
                {
                    v1 = t1[0];
                    v2 = t1[1];
                }
                if (t2[0] == v)
                {
                    v3 = t2[1];
                    v4 = t2[2];
                }
                else if (t2[1] == v)
                {
                    v3 = t2[0];
                    v4 = t2[2];
                }
                else
                {
                    v3 = t2[0];
                    v4 = t2[1];
                }
                if ((v1 == v3 && v2 == v4) || (v1 == v4 && v2 == v3))
                    continue; // same triangle
                if (abs(v1 - v2) > 1 || abs(v3 - v4) > 1)
                    continue; // non-consecutive edges for the opposite two edges
                if (((_vertices[v1]->position - _vertices[v]->position).dot(_vertices[v2]->binormal)) * ((_vertices[v3]->position - _vertices[v]->position).dot(_vertices[v2]->binormal)) > 0)
                {
                    // same side of the stroke
                    // now check for projective overlap: v3 and v4 onto v1 and v2
                    bool overlap_res = checkOverlap(v, v1, v2, v3, v4);
                    if (overlap_res)
                    {
                        undecided_vertices.insert(v);

                        // add incompatible triangles to map
                        if (!this->vertexIncompatibleTrianglesMap.contains(v))
                        {
                            this->vertexIncompatibleTrianglesMap[v] = emptySet;
                        }
                        unordered_set<pair<Vector3i, Vector3i>, classcomp2> incompatibleTriangleSet = this->vertexIncompatibleTrianglesMap.at(v);
                        if (!incompatibleTriangleSet.contains(make_pair(t1, t2)) && !incompatibleTriangleSet.contains(make_pair(t2, t1)))
                        {
                            incompatibleTriangleSet.insert(make_pair(t1, t2));
                            this->vertexIncompatibleTrianglesMap[v] = incompatibleTriangleSet;
                        }
                    }
                }
            }
        }
    }
    std::cout << "Vertex shared: overlap: " << undecided_vertices.size() << " out of total of " << _vertices.size() << " vertices." << std::endl;
    // combine all clusters and return it?
    //    for (auto edge : undecided_edges)
    //    {
    //        res.push_back(edgeToTriangles.at(edge));
    //    }
    //    for (auto vertex : undecided_vertices)
    //    {
    //        res.push_back(vertexToTriangles.at(vertex));
    //    }
    //    undecidedTriangles = res;

    // NOTE: undecided_edges can be transformed into an unordered_set for faster performance
    while (undecided_edges.size() + undecided_vertices.size() > 0)
    {
        cout << "loop with size " << undecided_edges.size() + undecided_vertices.size() << endl;
        unordered_set<Vector3i, classcomp3> undecidedTrianglesCluster;
        unordered_set<pair<Vector3i, Vector3i>, classcomp2> incompatibleTriangles; // that exist in this cluster

        deque<pair<int, int>> unprocessed_undecided_edges;
        deque<int> unprocessed_undecided_vertices;

        // first item in either of the above deques
        if (undecided_edges.size())
        {
            pair<int, int> first_undecided_edge = *undecided_edges.begin();
            unprocessed_undecided_edges.push_back(first_undecided_edge);
        }
        else
        {
            int first_undecided_vertex = *undecided_vertices.begin();
            unprocessed_undecided_vertices.push_back(first_undecided_vertex);
        }

        while (unprocessed_undecided_edges.size() + unprocessed_undecided_vertices.size())
        {
            cout << "subloop with size " << unprocessed_undecided_edges.size() + unprocessed_undecided_vertices.size() << endl;
            while (unprocessed_undecided_edges.size())
            {
                cout << "unprocessed_undecided_edges subloop" << endl;
                // the edge
                pair<int, int> undecided_edge = unprocessed_undecided_edges.front();
                unprocessed_undecided_edges.pop_front();
                undecided_edges.erase(undecided_edge);

                auto undecided_triangles = edgeToTriangles.at(undecided_edge);
                undecidedTrianglesCluster.insert(undecided_triangles.begin(), undecided_triangles.end());
                auto cur_incompatible_triangles = edgeIncompatibleTrianglesMap.at(this->encodeEdge(undecided_edge.first, undecided_edge.second));
                incompatibleTriangles.insert(cur_incompatible_triangles.begin(), cur_incompatible_triangles.end());

                // the edge's vertices
                int undecided_vertex1 = undecided_edge.first;
                int undecided_vertex2 = undecided_edge.second;
                if (undecided_vertices.contains(undecided_vertex1))
                {
                    unprocessed_undecided_vertices.push_back(undecided_vertex1);
                }
                if (undecided_vertices.contains(undecided_vertex2))
                {
                    unprocessed_undecided_vertices.push_back(undecided_vertex2);
                }
            }

            while (unprocessed_undecided_vertices.size())
            {
                cout << "unprocessed_undecided_vertices subloop" << endl;
                // the vertex
                int undecided_vertex = unprocessed_undecided_vertices.front();
                unprocessed_undecided_vertices.pop_front();
                undecided_vertices.erase(undecided_vertex);

                auto undecided_triangles = vertexToTriangles.at(undecided_vertex);
                undecidedTrianglesCluster.insert(undecided_triangles.begin(), undecided_triangles.end());
                auto cur_incompatible_triangles = vertexIncompatibleTrianglesMap.at(undecided_vertex);
                incompatibleTriangles.insert(cur_incompatible_triangles.begin(), cur_incompatible_triangles.end());

                // the edges surrounding the vertex
                // go through each undecided edge to find ones that has this vertex as its end
                for (const pair<int, int> &edge : undecided_edges)
                {
                    if (edge.first == undecided_vertex || edge.second == undecided_vertex)
                    {
                        unprocessed_undecided_edges.push_back(edge);
                    }
                }
            }
            cout << "end of subloop with size " << unprocessed_undecided_edges.size() + unprocessed_undecided_vertices.size() << endl;
        }

        cout << "start creating cluster" << endl;
        vector<Vector3i> undecidedTrianglesClusterVector(undecidedTrianglesCluster.begin(), undecidedTrianglesCluster.end());
        unordered_map<Vector3i, int, classcomp3> triangleToIndexMap;
        for (int i = 0; i < undecidedTrianglesClusterVector.size(); i++)
        {
            triangleToIndexMap[undecidedTrianglesClusterVector[i]] = i;
        }
        unordered_map<int, unordered_set<int>> incompatibleTrianglesMap;
        for (const pair<Vector3i, Vector3i> &trianglePair : incompatibleTriangles)
        {
            int triangle1 = triangleToIndexMap.at(trianglePair.first);
            int triangle2 = triangleToIndexMap.at(trianglePair.second);

            if (!incompatibleTrianglesMap.contains(triangle1))
            {
                unordered_set<int> emptyTriangleSet;
                incompatibleTrianglesMap[triangle1] = emptyTriangleSet;
            }
            unordered_set<int> triangle1Incompatibles = incompatibleTrianglesMap.at(triangle1);
            triangle1Incompatibles.insert(triangle2);
            incompatibleTrianglesMap[triangle1] = triangle1Incompatibles;

            if (!incompatibleTrianglesMap.contains(triangle2))
            {
                unordered_set<int> emptyTriangleSet;
                incompatibleTrianglesMap[triangle2] = emptyTriangleSet;
            }
            unordered_set<int> triangle2Incompatibles = incompatibleTrianglesMap.at(triangle2);
            triangle2Incompatibles.insert(triangle1);
            incompatibleTrianglesMap[triangle2] = triangle2Incompatibles;
        }
        this->undecidedTriangles.push_back(undecidedTrianglesClusterVector);
        this->incompatibleTriangles.push_back(incompatibleTrianglesMap);
        cout << "end creating cluster, with size " << undecided_edges.size() + undecided_vertices.size() << endl;
    }
    cout << "end creating all clusters" << endl;
}

bool Mesh::checkOverlap(int v, int v1, int v2, int v3, int v4)
{
    Vector3f v1v = _vertices[v]->position - _vertices[v1]->position;
    Vector3f v2v = _vertices[v]->position - _vertices[v2]->position;
    Vector3f normal = v1v.cross(v2v); // normal of the triangle plane
    // v3, v4 should be on the opposite side of v2 w.r.t. v1v
    Vector3f b_v1v = v1v.cross(normal); // binormal of v1v
    Vector3f b_v2v = v2v.cross(normal); // binormal of v2v
    // three cases, with v3 and v4 interchangeable
    Vector3f v1v3 = _vertices[v3]->position - _vertices[v1]->position;
    Vector3f v1v4 = _vertices[v4]->position - _vertices[v1]->position;
    Vector3f v1v2 = _vertices[v2]->position - _vertices[v1]->position;

    Vector3f v2v3 = _vertices[v3]->position - _vertices[v2]->position;
    Vector3f v2v4 = _vertices[v4]->position - _vertices[v2]->position;
    Vector3f v2v1 = _vertices[v1]->position - _vertices[v2]->position;

    int count_false = 0;
    // Case 1 -- if all four booleans are true
    // v3 same v2 w.r.t. v1v  --> v1v3 and v1v2 same direction --> v1v3.dot(b_v1v) same sign as v1v2.dot(b_v1v)
    bool v3_v2_v1v = v1v3.dot(b_v1v) * v1v2.dot(b_v1v) > 0;
    if (!v3_v2_v1v)
        count_false++;
    // v4 same v2 w.r.t. v1v
    bool v4_v2_v1v = v1v4.dot(b_v1v) * v1v2.dot(b_v1v) > 0;
    if (!v4_v2_v1v)
        count_false++;
    // v3 same v1 w.r.t.v2v
    bool v3_v1_v2v = v2v3.dot(b_v2v) * v2v1.dot(b_v2v) > 0;
    if (!v3_v1_v2v)
        count_false++;
    // v4 same v1 w.r.t.v2v
    bool v4_v1_v2v = v2v4.dot(b_v2v) * v2v1.dot(b_v2v) > 0;
    if (!v4_v1_v2v)
        count_false++;

    // case 1: true, true, true, true -- 0 false
    // case 2,3,4,5: only 1 false
    return count_false <= 1;
}

// --------------------- Section 6 ------------------------
// --------------------- Get boundary matches ------------------------
// A function that groups the vertices into components (connected partial surfaces)
vector<vector<Vertex*>> Mesh::getComponents() {
    // use the adjacency list to get components? should generate a vector<vector<int>> for a group of components
    // dfs to get each component
    int n = _vertices.size();
    vector<bool> visited(n, false);
    vector<vector<Vertex*>> components;
    int num_components = 0;
    for (int i = 0; i < _vertices.size(); i++) {
        if(!visited[i]) {
            // open a new vector to keep track of this new component
            vector<Vertex*> component;
            component.push_back(_vertices[i]);
            _vertices[i]->componentIndex = num_components; // use the current number to index the vertex's component -- theoretically only need this
            dfs(adjacencyList, visited, i, component, num_components); // this should populate the component
            components.push_back(component);
            num_components++;
        }
    }
    std::cout << "Here in getComponents, total number of components is: " << num_components << std::endl;
    return components;
}
// have: unordered_map<int, vector<int>> adjacencyList
void Mesh::dfs(unordered_map<int, vector<int>> &adj, vector<bool> &visited, int src, vector<Vertex*> &component, int num_components) {
    visited[src] = true;
    for(int i : adj[src]) {
        if(!visited[i]) {
            component.push_back(_vertices[i]);
            _vertices[i]->componentIndex = num_components; // theoretically only need this
            dfs(adj, visited, i, component, num_components);
        }
    }
}

// now we can process each component individually and calculate its average match distance -- then store the distance in the Vertex too
// first we need an easy way to identify which component a vertex belongs to -- assign a number? Yes suppose we already have this in componentIndex
// just loop through all vertices and accumulate match distances
void Mesh::getComponentMatchDistance() {
    // make a map out of all componentIndex to the number of matches it contains, and its total sum of matching distance
    unordered_map<int, pair<int, float>> componentIndex_dist;
    for (int i = 0; i < _vertices.size(); i++) {
        // std::cout << _vertices[i]->componentIndex << std::endl;
        float matchDist = 0.0; // calculate the match distance from the vertex to its match -- avg of left and right matches
        Vector3f pos1 = _vertices[i]->position;
        if (leftMatch[i] != -1) {
            Vector3f pos2 = _vertices[leftMatch[i]]->position;
            matchDist = (pos1 - pos2).norm();
            if (componentIndex_dist.contains(_vertices[i]->componentIndex)) {
                componentIndex_dist[_vertices[i]->componentIndex].first++;
                componentIndex_dist[_vertices[i]->componentIndex].second += matchDist;
            }else {
                // componentIndex_dist[_vertices[i]->componentIndex] = make_pair(1, matchDist);
                componentIndex_dist.insert({_vertices[i]->componentIndex, make_pair(1, matchDist)});
            }
        }
        if (rightMatch[i] != -1) {
            Vector3f pos3 = _vertices[rightMatch[i]]->position;
            matchDist = (pos1 - pos3).norm();
            if (componentIndex_dist.contains(_vertices[i]->componentIndex)) {
                componentIndex_dist[_vertices[i]->componentIndex].first++;
                componentIndex_dist[_vertices[i]->componentIndex].second += matchDist;
            }else {
                // componentIndex_dist[_vertices[i]->componentIndex] = make_pair(1, matchDist);
                componentIndex_dist.insert({_vertices[i]->componentIndex, make_pair(1, matchDist)});
            }
        }
        if (leftMatch[i] == -1 && rightMatch[i] == -1) { // the point is isolated for now? What do we do?
            // std::cout << _vertices[i]->componentIndex << std::endl;
            continue; // just continue to the next vertex
        }
    }
    // finally populate the map of component index to its average distance to be referenced by boundry vertices
    for (const auto &myPair : componentIndex_dist) { // key, value = component index, pair of (num, sum_distance)
        // componentIndex_avgDist[myPair.first] = myPair.second.second / (float) myPair.second.first;
        componentIndex_avgDist.insert({myPair.first, myPair.second.second / (float) myPair.second.first});
    }
}

// --------------------- Section 6 ------------------------
// --------------------- start matching ------------------------
// use componentIndex_avgDist and Vertex->componentIndex to get the corresponding average distance
// for each boundary line, need vector<vector<int>> boundary_candidates;
void Mesh::getBoundaryCandidates() {
    // first compute components and populate dmax for each component: the map componentIndex_avgDist
    vector<vector<Vertex*>> components = getComponents();
    getComponentMatchDistance();
    getNewBoundaryTangentsAndBinormals();

    // for each boundary vertex, loop through all other boundary vertices to get matches
    // vector<pair<bool,vector<int>>> _boundaries
    // If the vector represents a line, the bool is true. If the vector represents a cycle, the bool is false
    // For cycles, the start index is listed at the end as well
    for (auto & boundary_line : _boundaries) {
        for (int i = 0; i < boundary_line.second.size(); i++) {
            int boundary_vertex_p = boundary_line.second[i];
            int neighbor1 = -1; int neighbor2 = -1;
            if (boundary_line.first) { // it's a line
                if (i == 0) {
                    neighbor1 = boundary_line.second[i+1];
                } else if (i == boundary_line.second.size()-1) {
                    neighbor1 = boundary_line.second[i-1];
                } else {
                    neighbor1 = boundary_line.second[i-1];
                    neighbor2 = boundary_line.second[i+1];
                }
            }else { // it's a cycle, then everyone has two neighbors
                if (i == 0 || i == boundary_line.second.size()-1) {
                    neighbor1 = boundary_line.second[i+1];
                    neighbor2 = boundary_line.second[boundary_line.second.size()-2]; // second to last
                } else {
                    neighbor1 = boundary_line.second[i-1];
                    neighbor2 = boundary_line.second[i+1];
                }
            }
            // side product: a boundary neighbor map that will be useful in computeM_boundary
            boundaryNeighbors.insert({boundary_vertex_p, {neighbor1, neighbor2}});

            // loop through all other boundary vertices to check candidacy: except itself and its two neighbors on the same boundary stroke
            // populate unordered_map<int, unordered_set<int>> boundaryRestrictedMatchingCandidates
            // use boundary_vertex_p as the key
            for (auto & boundary_line_q : _boundaries) {
                for (int j = 0; j < boundary_line_q.second.size(); j++) {
                    int boundary_vertex_q = boundary_line_q.second[j];

                    // Check the 3 matching conditions for p and q
                    // Condition 3: p and q cannot be the same and cannot be neighbors
                    if (boundary_vertex_q == boundary_vertex_p || boundary_vertex_q == neighbor1 || boundary_vertex_q == neighbor2) continue;
                    // Condition 2: angle smaller than 80
                    Vertex *p = this->_vertices[boundary_vertex_p]; Vertex *q = this->_vertices[boundary_vertex_q];
                    Vector3f pq = (q->position - p->position).normalized();
                    float angle = acos(pq.dot(p->boundary_binormal));
                    if (angle > M_PI / 180.f * 80.f) continue;
                    // Condition 1: length longer than dmax
                    float length = pq.norm();
                    float pStrokeWidth = p->strokeWidth; float qStrokeWidth = q->strokeWidth;
                    float sigma = 1.38f/2.f*(pStrokeWidth + qStrokeWidth); // default: 1.5f/2.f*(pStrokeWidth + qStrokeWidth) -- 191 matches
                    float dmax = componentIndex_avgDist[p->componentIndex]; // is this correct?
                    if (length > sigma) continue;
                    // if passes all conditions, add to the set
                    if (!boundaryRestrictedMatchingCandidates.contains(boundary_vertex_p)){
                        boundaryRestrictedMatchingCandidates.insert({boundary_vertex_p, unordered_set<int>()});
                    }
                    unordered_set<int> candidateSet = boundaryRestrictedMatchingCandidates.at(boundary_vertex_p);
                    candidateSet.insert(boundary_vertex_q);
                    boundaryRestrictedMatchingCandidates[boundary_vertex_p] = candidateSet;
                }
            }
        }
    }

}


void Mesh::getBoundaryMatches() {
    // process each boundary
    for (auto & boundary_line : _boundaries) {
        vector<int> S = boundary_line.second;
        // convert the unordered_map<int, unordered_set<int>> boundaryRestrictedMatchingCandidates into a vector<vector<int>> candidates for S
        vector<vector<int>> candidates;
        for (int point : S) {
            vector<int> point_candidates = {};
            if (boundaryRestrictedMatchingCandidates.contains(point)) { // if it has some potential matches
                point_candidates.insert(point_candidates.end(), boundaryRestrictedMatchingCandidates.at(point).begin(), boundaryRestrictedMatchingCandidates.at(point).end());
            }
            candidates.push_back(point_candidates);
        }
        // ========== check for previous matches and adjust candidate sets
        for (int i = 0; i < S.size(); i++) {
            int point = S[i];
            if (currentBoundaryMatches.contains(point)) {
                for (int potential_match : currentBoundaryMatches.at(point)) {
                    bool hasMatchBefore = false;
                    if (hasMatchBefore) {
                        candidates[i].push_back(potential_match);
                    } else {
                        candidates[i] = {potential_match};
                        hasMatchBefore = true;
                    }
                }
            }
        }
        // starting finding matches for S -- every match should be a right match
        vector<int> boundary_matches = viterbi(S, candidates, false, true);
        // populate unordered_map<int, int> boundaryMatch
        for (int i = 0; i < S.size(); i++)
        {
            boundaryMatch.insert({S[i], boundary_matches[i]});

            // ========= populate current matches
            if (currentBoundaryMatches.contains(boundary_matches[i])) {
                currentMatches.at(boundary_matches[i]).push_back(S[i]);
            }else {
                currentMatches.insert({boundary_matches[i], {S[i]}});
            }
        }
    }
    int count_matches = 0;
    for (auto i = boundaryMatch.begin(); i != boundaryMatch.end(); i++) {
        if (i->second != -1) count_matches++;
    }
    std::cout << "Number of added boundary matches: " << count_matches << std::endl;
}

// --------------------- Get matches functions ------------------------
void Mesh::calculateTangentsAndBinormals(const vector<Vector3f> &vertices, const vector<Vector3f> &vertexNormals)
{
    // loop through all lines
    for (auto &line : _lines)
    {
        int n = line.size();
        // loop through all vertices on the line
        Vector3f first_segment = (vertices[line[1]] - vertices[line[0]]).normalized(); // tangent of the first vertex is just the line segment direction
        // ===== changed first tangent: need to project it onto the orthogonal direction of the normal!!
        Vector3f cur_normal = vertexNormals[line[0]];
        Vector3f first_segment_parallel = first_segment.dot(cur_normal) / (cur_normal.norm() * cur_normal.norm()) * cur_normal;
        Vector3f first_tangent = (first_segment - first_segment_parallel).normalized();
        _vertices[line[0]]->tangent = first_tangent;
        _vertices[line[0]]->binormal = (first_tangent.cross(vertexNormals[line[0]])).normalized();

        for (int i = 1; i < n - 1; i++)
        {
            Vector3f curA = vertices[line[i - 1]];
            Vector3f curB = vertices[line[i]];
            Vector3f curC = vertices[line[i + 1]];
            Vector3f AC = curC - curA;
            Vector3f B_normal = vertexNormals[line[i]];
            Vector3f AC_parallel = AC.dot(B_normal) / (B_normal.norm() * B_normal.norm()) * B_normal; // AC projected onto the direction of normal

            Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
            _vertices[line[i]]->tangent = cur_tangent;
            _vertices[line[i]]->binormal = (cur_tangent.cross(vertexNormals[line[i]])).normalized();
        }
        Vector3f last_segment = (vertices[line[n - 1]] - vertices[line[n - 2]]).normalized(); // tangent of the last vertex is just the line segment direction
        // ===== changed last tangent: need to project it onto the orthogonal direction of the normal!!
        cur_normal = vertexNormals[line[n-1]];
        Vector3f last_segment_parallel = last_segment.dot(cur_normal) / (cur_normal.norm() * cur_normal.norm()) * cur_normal;
        Vector3f last_tangent = (last_segment - last_segment_parallel).normalized();
        _vertices[line[n - 1]]->tangent = last_tangent;
        _vertices[line[n - 1]]->binormal = (last_tangent.cross(vertexNormals[line[n - 1]])).normalized();
    }
}

float Mesh::vertexVertexScore(Vertex *P, Vertex *Q, bool leftside)
{
    Vector3f p = P->position;
    Vector3f q = Q->position;
    Vector3f tp = P->tangent;
    Vector3f tq = Q->tangent;
    Vector3f pb = (tp.cross(P->normal)).normalized();
    Vector3f qb = (tq.cross(Q->normal)).normalized();
    float da = (p - q).norm();
    float dt = 0.5 * (abs((p - q).dot(tp)) + abs((p - q).dot(tq)));
    float pStrokeWidth = P->strokeWidth;
    float qStrokeWidth = Q->strokeWidth;
    // This is specifically for a left match.
    Vector3f pc;
    if (leftside)
    {
        pc = p - pStrokeWidth * pb;
    }
    else
    {
        pc = p + pStrokeWidth * pb;
    }

    Vector3f ql = q - qStrokeWidth * qb;
    Vector3f qr = q + qStrokeWidth * qb;

    Vector3f qc;
    if ((ql - pc).norm() < (qr - pc).norm())
    {
        qc = ql;
    }
    else
    {
        qc = qr;
    }
    Vector3f mpqprime = 0.5 * (pc + qc);
    Vector3f mpq = 0.5 * (p + q);
    float dln = (mpq - mpqprime).norm();
    float sigma = 1.5f / 2.f * (pStrokeWidth + qStrokeWidth);
    float finalscore = exp(-pow(da + dt + dln, 2) / (2.f * pow(sigma, 2.f)));
    return finalscore;
}

// Pi_1 mean P_(i+1)
float Mesh::persistenceScore(Vertex *Pi, Vertex *Qi, Vertex *Pi_1, Vertex *Qi_1)
{
    Vector3f pi_1 = Pi_1->position;
    Vector3f qi_1 = Qi_1->position;
    Vector3f pi = Pi->position;
    Vector3f qi = Qi->position;
    float pStrokeWidth = Pi->strokeWidth;
    float qStrokeWidth = Qi->strokeWidth;
    float dp = ((pi_1 - pi) - (qi_1 - qi)).norm() + ((pi_1 - qi) - (qi_1 - pi)).norm() + ((pi_1 - qi_1) - (pi - qi)).norm();

    float sigma = 1.5f / 2.f * (pStrokeWidth + qStrokeWidth);
    float finalscore = exp(-pow(dp, 2) / (2.f * pow(sigma, 2.f)));
    return finalscore;
}

// Intermediate M from step i-1 to i
// pi_1 means p_(i-1)
float Mesh::computeM(int pi, int qi, int pi_1, int qi_1, bool leftSide)
{
    float vv = vertexVertexScore(_vertices[pi_1], _vertices[qi_1], leftSide);
    float pers = persistenceScore(_vertices[pi_1], _vertices[qi_1], _vertices[pi], _vertices[qi]);

    // naively punish inconsecutive qi and qi-1
    if (abs(qi - qi_1) > 1)
        return 1e-10 * vv * pers;
    return vv * pers;
}

float Mesh::computeM_boundary(int pi, int qi, int pi_1, int qi_1, bool leftSide)
{
    float vv = vertexVertexScore(_vertices[pi_1], _vertices[qi_1], leftSide);
    float pers = persistenceScore(_vertices[pi_1], _vertices[qi_1], _vertices[pi], _vertices[qi]);

    // naively punish inconsecutive qi and qi-1 -- now punish inconsecutive boundary points
    // if (boundaryNeighbors.at(qi)[0] != qi_1 && boundaryNeighbors.at(qi)[1] != qi_1) return 1e-10*vv*pers;
    // if (abs(qi - qi_1) > 1) return 1e-10*vv*pers;
    return vv * pers;

}

/**
 * @brief perform one viterbi on one stroke S to find the sequence of q's that maximizes the objective function of M_l
 * @param vector<Vertex*> S: stroke to be processed
 * @param vector<vector<Vertex*>> candidates: a vector of candidates vector<C(pi)> for each vertex pi in S
 * @return vector<Vertex*> Q: sequence of q's, each qi is the optimal match for each pi in S
 */
vector<int> Mesh::viterbi(vector<int> S, vector<vector<int>> candidates, bool leftSide, bool boundary)
{
    int k = 1;        // time index
    int K = S.size(); // number of steps = number of vertices in S
    int M = 0;        // max of number of candidates among all points pi
    for (int i = 0; i < candidates.size(); i++)
    {
        if (candidates[i].size() > M)
        {
            M = candidates[i].size();
        }
    }
    // if (M == 0) return {}; // if S does not have any potential matches on this side
    // construct a float[][] of size M*K
    float scores[M][K];
    // initialize the first column of dp to be all ones -- candidates for p0
    for (int i = 0; i < candidates[0].size(); i++)
    {
        scores[i][0] = 1.0;
    }
    // also need to store the current sequence of states at each q at time k -- an array of vectors, the array is of fixed size M
    vector<vector<int>> prev_sequences;
    vector<vector<int>> cur_sequences;
    // initialize the sequences to contain the starting points -- candidates for p0
    for (int i = 0; i < candidates[0].size(); i++)
    {
        prev_sequences.push_back({candidates[0][i]});
    }
    if (candidates[0].size() == 0)
    { // first node does not have candidate matches
        prev_sequences.push_back({-1});
    }
    // begin iterating through each step
    int prev_k = 0; // the previous step with valid states (candidates nonempty)
    while (k < K)
    {
        if (candidates[k].size() == 0)
        { // if the current node does not have candidate matches
            // add -1 to each prev_sequence to indicate that it does not have a matching vertex
            // then continue to next k without setting prev_k -- prev_k should be the last valid k
            if (prev_sequences.empty())
            {
                prev_sequences.push_back({-1});
            }
            else
            {
                for (auto &seq : prev_sequences)
                {
                    seq.push_back(-1);
                }
            }

            k++;
            continue;
        }
        cur_sequences = {};
        for (int cur = 0; cur < candidates[k].size(); cur++)
        {
            // for each current q, select the prev that maximizes M_score so far
            // for prev, its M_score is stored in scores[prev][k-1]
            float cur_max = -1e36;
            int max_prev = 0;
            for (int prev = 0; prev < candidates[prev_k].size(); prev++)
            {
                float stepM = 0.0;
                if (boundary) stepM = computeM_boundary(S[k], candidates[k][cur], S[prev_k], candidates[prev_k][prev], leftSide);
                else stepM = computeM(S[k], candidates[k][cur], S[prev_k], candidates[prev_k][prev], leftSide);

                if (stepM * scores[prev][prev_k] > cur_max)
                {
                    cur_max = stepM * scores[prev][prev_k];
                    max_prev = prev;
                }
            }
            vector<int> newvec = prev_sequences[max_prev];
            newvec.push_back(candidates[k][cur]);
            cur_sequences.push_back(newvec); // extend from prev_seqence by appending cur to the mas prev sequence
            // use cur_max as the score for cur
            scores[cur][k] = cur_max;
        }
        // updaet prev_sequences to be cur_sequences: prev_sequences = cur_sequences
        prev_sequences = {};
        for (int i = 0; i < cur_sequences.size(); i++)
        {
            prev_sequences.push_back(cur_sequences[i]);
        }
        prev_k = k;
        k++;
    }
    // now select the q that has the maximum score and retrieve its sequence
    // all final scores are in scores[][K-1]
    int final_index = 0;
    float max_score = 0;
    for (int i = 0; i < prev_sequences.size(); i++)
    {
        if (scores[i][K - 1] > max_score)
        {
            max_score = scores[i][K - 1];
            final_index = i;
        }
    }
    return prev_sequences[final_index];
}

// populate the unordered_map<int, int> leftMatch and rightMatch
void Mesh::getMatches()
{
    // process each strip
    for (vector<int> S : _lines)
    {
        // convert the unordered_map<int, unordered_set<int>> leftRestrictedMatchingCandidates into two vector<vector<int>> candidates for S
        vector<vector<int>> left_candidates;
        vector<vector<int>> right_candidates;
        for (int point : S)
        {
            vector<int> point_left_candidates = {};
            if (leftRestrictedMatchingCandidates.contains(point))
            { // if it has some potential left matches
                point_left_candidates.insert(point_left_candidates.end(), leftRestrictedMatchingCandidates.at(point).begin(), leftRestrictedMatchingCandidates.at(point).end());
            }
            left_candidates.push_back(point_left_candidates);

            vector<int> point_right_candidates = {};
            if (rightRestrictedMatchingCandidates.contains(point))
            { // if it has some potential right matches
                point_right_candidates.insert(point_right_candidates.end(), rightRestrictedMatchingCandidates.at(point).begin(), rightRestrictedMatchingCandidates.at(point).end());
            }
            right_candidates.push_back(point_right_candidates);
        }
        // ========== check for previous matches and adjust candidate sets
        for (int i = 0; i < S.size(); i++)
        {
            int point = S[i];
            if (currentMatches.contains(point))
            {
                for (int potential_match : currentMatches.at(point))
                {
                    // see if it's on the left or right side of point using the binormal
                    Vector3f binormal = _vertices[point]->tangent.cross(_vertices[point]->normal);
                    bool hasRightBefore = false;
                    bool hasLeftBefore = false;
                    // if it lines up with the binormal it's on right side, should go into right candidates
                    if ((_vertices[potential_match]->position - _vertices[point]->position).dot(binormal) > 0)
                    { // right side
                        if (hasRightBefore)
                        {
                            right_candidates[i].push_back(potential_match);
                        }
                        else
                        {
                            right_candidates[i] = {potential_match};
                            hasRightBefore = true;
                        }
                    }
                    else
                    { // left side
                        if (hasLeftBefore)
                        {
                            left_candidates[i].push_back(potential_match);
                        }
                        else
                        {
                            left_candidates[i] = {potential_match};
                            hasLeftBefore = true;
                        }
                    }
                }
            }
        }

        // starting finding matches for S
        vector<int> left_matches = viterbi(S, left_candidates, true, false);
        vector<int> right_matches = viterbi(S, right_candidates, false, false);

        // populate unordered_map<int, int> leftMatch and rightMatch
        for (int i = 0; i < S.size(); i++)
        {
            leftMatch.insert({S[i], left_matches[i]});
            rightMatch.insert({S[i], right_matches[i]});

            // ========= populate current matches
            if (currentMatches.contains(left_matches[i]))
            {
                currentMatches.at(left_matches[i]).push_back(S[i]);
            }
            else
            {
                currentMatches.insert({left_matches[i], {S[i]}});
            }
            if (currentMatches.contains(right_matches[i]))
            {
                currentMatches.at(right_matches[i]).push_back(S[i]);
            }
            else
            {
                currentMatches.insert({right_matches[i], {S[i]}});
            }
        }
    }
}

// -------- Get matching candidates functions -------------------------------------------------------------------------------

/**
 * @brief Mesh::splitStrokesIntoLeftRight
 * @param baseStrokeIndex
 * @return (LEFT strokes, RIGHT strokes)
 */
pair<vector<int>, vector<int>> Mesh::splitStrokesIntoLeftRight(int baseStrokeIndex)
{
    vector<int> leftStrokeIndices;
    vector<int> rightStrokeIndices;

    vector<int> baseStroke = this->_lines[baseStrokeIndex];
    int baseStrokeSize = baseStroke.size();
    int baseStrokeMidpoint = (int)((baseStrokeSize - (baseStrokeSize % 2)) / 2);
    Vertex *baseStrokeMidpointVertex = this->_vertices[baseStroke[baseStrokeMidpoint]];
    Vector3f baseBinormal = (baseStrokeMidpointVertex->tangent.cross(baseStrokeMidpointVertex->normal)).normalized();

    for (int i = 0; i < this->_lines.size(); i++)
    {
        if (i == baseStrokeIndex)
            continue;

        // get midpoint
        vector<int> currStroke = this->_lines[i];
        int currStrokeSize = currStroke.size();
        int currStrokeMidpoint = (int)((currStrokeSize - (currStrokeSize % 2)) / 2);
        int currStrokeMidpointInd = currStroke[currStrokeMidpoint];
        Vertex *currStrokeMidpointVertex = this->_vertices[currStrokeMidpointInd];
        Vector3f baseToCurrVec = (currStrokeMidpointVertex->position - baseStrokeMidpointVertex->position).normalized();

        float dotProduct = baseBinormal.dot(baseToCurrVec);
        if (dotProduct > 0)
        {
            rightStrokeIndices.push_back(i);
        }
        else
        {
            leftStrokeIndices.push_back(i);
        }
    }

    return make_pair(leftStrokeIndices, rightStrokeIndices);
}

bool areVerticesImmediateNeighbors(int pIndex, int qIndex, vector<int> stroke)
{
    for (int i = 0; i < stroke.size(); i++)
    {
        if (i == pIndex)
        {
            return qIndex == i - 1 || qIndex == i + 1;
        }
    }
    return false;
}

/**
 * @brief Mesh::intabs
 * @param x
 * @return the absolute value of x
 */
int intabs(int x)
{
    if (x < 0)
    {
        return -x;
    }
    return x;
}

/**
 * @brief Mesh::getnorm
 * @param p0
 * @param p1
 * @param p2
 * @return the normal of the plane formed by the three points
 */
Vector3f getnorm(Vector3f p0, Vector3f p1, Vector3f p2)
{
    Vector3f v01 = p1 - p0;
    Vector3f v02 = p2 - p0;
    Vector3f crossed = v01.cross(v02);
    return crossed / crossed.norm();
}

/**
 * @brief Mesh::triangulatePair
 * @param pi
 * @param qi
 * @param pn
 * @param qn
 * @return a list of triangles triangulating between the edges pi pn and qi qn
 */
std::vector<Vector3i> Mesh::triangulatePair(int pi, int qi, int pn, int qn)
{
    // So in practice, we want to push these new triangles back.
    // We need to be able to go through each pair of strokes, and iterate through all consecutive matches.

    std::vector<Vector3i> triangles = std::vector<Vector3i>();
    // qi and qi1 belong to the same stroke by default. Their difference should just be the number of points in between.
    // Check if qi and qn are the same
    if (qi == qn)
    {
        triangles.push_back(Vector3i(pi, pn, qi));
    }
    else
    {
        // qi and qn are not the same
        int n_midpoints = intabs(qn - qi) - 1;

        if (n_midpoints == 0)
        {
            // qi and qn are consecutive

            // Get positions of all points
            Vector3f pip = _vertices[pi]->position;
            Vector3f qip = _vertices[qi]->position;
            Vector3f pnp = _vertices[pn]->position;
            Vector3f qnp = _vertices[qn]->position;

            // Try both triangulations. Choose the one with a larger dihedral angle.
            // Edit: Dihedral angle might not be great. Just do the one with the smaller diagonal difference.
            if ((pip - qnp).norm() < (qip - pnp).norm())
            {
                triangles.push_back(Vector3i(pi, pn, qn));
                triangles.push_back(Vector3i(pi, qi, qn));
            }
            else
            {
                triangles.push_back(Vector3i(qi, qn, pn));
                triangles.push_back(Vector3i(qi, pi, pn));
            }
        }
        else
        {
            // //There are a nonzero number of midpoints
            // //For now, we choose a naive solution for triangulating. Splice along the approximate midpoint of the segment connecting qi to qn.
            int i_mid = static_cast<int>(floor(0.5 * (static_cast<float>(qi) + static_cast<float>(qn))));

            int ibegin;
            int iend;
            if (qi < qn)
            {
                ibegin = qi;
                iend = qn;
            }
            else
            {
                ibegin = qn;
                iend = qi;
            }

            // //Add above triangles
            for (int i = ibegin; i < i_mid; i++)
            {
                triangles.push_back(Vector3i(pi, i + 1, i));
            }
            // Add middle triangle
            // i_mid should be between qi and qn
            triangles.push_back(Vector3i(pi, pn, i_mid));
            // I HAVE A FEELING QI AND QN ARE NOT ON THE SAME STROKE

            // Add below triangles
            for (int i = i_mid; i < iend; i++)
            {
                triangles.push_back(Vector3i(pn, i + 1, i));
            }
        }
    }
    return triangles;
}

/**
 * @brief Mesh::doTwoVerticesMatch
 * @param p
 * @param q
 * @param leftside
 * @param isOnSameStroke - whether p & q are on the same stroke
 * @param strokeIndex - -1 if isOnSameStroke == false, stroke index if isOnSameStroke == true
 * @param closeToEnd - if either of these vertices are in the vicinity of stroke end-vertices
 * @return whether the vertices satisfy the condition
 */
bool Mesh::doTwoVerticesMatch(int pIndex, int qIndex, bool leftside, bool isOnSameStroke, int strokeIndex, bool closeToEnd)
{
    Vertex *p = this->_vertices[pIndex];
    Vertex *q = this->_vertices[qIndex];
    Vector3f pq = q->position - p->position;
    //    Vector3f pBinormal = (p->tangent.cross(p->normal)).normalized();

    // condition 1
    float length = pq.norm();
    float pStrokeWidth = p->strokeWidth;
    float qStrokeWidth = q->strokeWidth;

    float sigma = 1.5f / 2.f * (pStrokeWidth + qStrokeWidth);
    if (length > sigma)
    {
        return false;
    }

    // condition 2
    pq.normalize();
    float angle;
    if (leftside)
    {
        angle = acos(pq.dot(-p->binormal));
    }
    else
    {
        angle = acos(pq.dot(p->binormal));
    }
    if (angle > M_PI / 3.f)
    {
        return false;
    }

    // condition 3
    if (isOnSameStroke)
    {
        vector<int> stroke = this->_lines[strokeIndex];
        if (areVerticesImmediateNeighbors(pIndex, qIndex, stroke))
        {
            return false;
        }
    }

    // enforcing condition 2 for vertices in the vicinity of stroke end-vertices
    if (closeToEnd)
    {
        Vector3f qp = (q->position - p->position).normalized();
        float angle2;                             // angle between qp & q's binormal
        bool leftside2 = q->binormal.dot(qp) < 0; // whether p is on leftside of q
        if (leftside2)
        {
            angle2 = acos(qp.dot(-q->binormal));
        }
        else
        {
            angle2 = acos(qp.dot(q->binormal));
        }
        if (angle2 > M_PI / 3.f)
        {
            return false;
        }
    }

    return true;
}

// can optimize, by "caching" results
// return -1 if:
//  - matching frequency < 30%
//  - there is no pair of consecutive vertices on S that match a pair of consecutive vertices on T
int Mesh::calcNumberOfMatches(int baseStrokeIndex, int otherStrokeIndex, bool leftside)
{
    int baseCount = 0; // how many of base stroke's vertices got matched

    vector<int> baseStroke = this->_lines[baseStrokeIndex];
    vector<int> otherStroke = this->_lines[otherStrokeIndex];

    bool isConsecutivePair = false; // for 2nd returning -1 condition
    int prevBaseVertexIndex = -1;   // index in stroke; for 2nd returning -1 condition

    for (int i = 0; i < baseStroke.size(); i++)
    {
        int baseVertex = baseStroke[i];
        bool isEverMatched = false;
        for (int j = 0; j < otherStroke.size(); j++)
        {
            // <notice, the 3 lines below are old comments>
            // TODO: IS THIS PART CORRECT?
            // !leftside is incorrect bc we don't know what side baseVertex is on w.r.t otherVertex
            // this is for p.9, section 5.2, restricted matching set subsection

            // TODO: FOR NOW, I SAY WITHIN THE LAST 5 VERTICES, BOTH V1 & V2 NEED TO HOLD CONDITION 2
            bool isEitherVertexCloseToEnd = (i > baseStroke.size() - this->settings->noOfNearEndVerticesToConsider || i < this->settings->noOfNearEndVerticesToConsider) || (j > otherStroke.size() - this->settings->noOfNearEndVerticesToConsider || j < this->settings->noOfNearEndVerticesToConsider);

            int otherVertex = otherStroke[j];
            bool baseOthermatched = this->doTwoVerticesMatch(baseVertex, otherVertex, leftside, false, -1, isEitherVertexCloseToEnd);

            //            baseOthermatched = baseOthermatched && this->doTwoVerticesMatch(otherVertex, baseVertex, !leftside, false, -1);

            if (baseOthermatched)
            {
                isEverMatched = true;
                // this is not 2-way, because we will do the other way later when this func is called again
                if (!validVertexVertexMatch.contains(baseVertex))
                {
                    validVertexVertexMatch[baseVertex] = unordered_map<int, unordered_set<int>>();
                }
                if (!validVertexVertexMatch.at(baseVertex).contains(otherStrokeIndex))
                {
                    validVertexVertexMatch[baseVertex][otherStrokeIndex] = unordered_set<int>();
                }
                unordered_set<int> matches = validVertexVertexMatch.at(baseVertex).at(otherStrokeIndex);
                matches.insert(otherVertex);
                this->validVertexVertexMatch[baseVertex][otherStrokeIndex] = matches;

                // if we have not found any consecutive pair yet!
                // and baseVertex is not the first vertex on stroke
                // and prevVertex actually has any matches
                int prevBaseVertex = baseStroke[prevBaseVertexIndex];
                if (!isConsecutivePair && prevBaseVertexIndex != -1 && this->validVertexVertexMatch.contains(prevBaseVertex) && this->validVertexVertexMatch.at(prevBaseVertex).contains(otherStrokeIndex))
                {
                    bool isBaseConsecutive = (i - prevBaseVertexIndex == 1);
                    unordered_set<int> prevBaseVertexMatches = this->validVertexVertexMatch.at(prevBaseVertex).at(otherStrokeIndex);
                    // this line below is saying:
                    // does the previous base vertex match with a consecutive neighbor of otherVertex?
                    bool isOtherConsecutive = prevBaseVertexMatches.contains(otherVertex - 1) || prevBaseVertexMatches.contains(otherVertex + 1);

                    isConsecutivePair = isBaseConsecutive && isOtherConsecutive;
                }
            }
        }
        if (isEverMatched)
        {
            baseCount++;
        }

        prevBaseVertexIndex = i;
    }

    float matchingFrequency = ((float)baseCount / (float)baseStroke.size()) > 0.3;

    baseCount = (matchingFrequency && isConsecutivePair) ? baseCount : -1;

    return baseCount;
}

vector<Vector3i> Mesh::outputTriangles()
{
    // guarantee:
    // ---- any triangle's vertices are in the same order as they should be in _faces

    // create a set of all undecided triangles (might be duplicated!)
    map<tuple<int, int, int>, bool> undecidedTrianglesMap;

    for (const vector<Vector3i> &graph : this->undecidedTriangles)
    {
        for (const Vector3i &triangle : graph)
        {
            undecidedTrianglesMap[make_tuple(triangle[0], triangle[1], triangle[2])] = true;
        }
    }

    // get only output triangles
    vector<Vector3i> outputs;
    for (const Vector3i &triangle : _faces)
    {
        if (!undecidedTrianglesMap.contains(make_tuple(triangle[0], triangle[1], triangle[2])))
        {
            outputs.push_back(triangle);
            this->outputTrianglesEdges.insert(this->encodeEdge(triangle[0], triangle[1]));
            this->outputTrianglesEdges.insert(this->encodeEdge(triangle[1], triangle[2]));
            this->outputTrianglesEdges.insert(this->encodeEdge(triangle[0], triangle[2]));
        }
    }
    return outputs;
}

int Mesh::encodeEdge(int item1, int item2)
{
    return utils::elegantPair(min(item1, item2), max(item1, item2));
}

void Mesh::createEdgePriorityQueue(unordered_map<int, vector<pair<float, int>>> adjacencies)
{
    this->edgePriorityQueue.clear(); // just to be safe!
    this->edgeCostMap.clear();

    for (auto &it : adjacencies)
    {
        int startVertex = it.first;
        vector<pair<float, int>> edges = it.second;
        for (auto edge : edges)
        {
            float cost = edge.first;
            int endVertex = edge.second;
            int encodedEdge = this->encodeEdge(startVertex, endVertex);
            if (this->edgeCostMap.contains(encodedEdge))
                continue;

            this->edgeCostMap[encodedEdge] = cost;
            pair<float, int> item = make_pair(cost, encodedEdge);
            this->edgePriorityQueue.insert(item);
        }

        this->unionFindParentMap[startVertex] = startVertex;
    }
}

/**
 * @brief Mesh::GAEC
 * @param adjacencies
 * ref: https://arxiv.org/pdf/1505.06973
 */
void Mesh::GAEC(unordered_map<int, vector<pair<float, int>>> adjacencies)
{
    // NOTE:
    // VERTICES MENTIONED IN THIS FUNCTION DOES NOT REFER TO ACTUAL VERTICES
    // THEY ARE COMPONENTS IN THE GRAPH
    // I also never delete any entries in this->edgeCostMap because that's unnecessary

    // construct priority queue for all edges
    this->createEdgePriorityQueue(adjacencies);

    while (!this->edgePriorityQueue.empty())
    {
        // cout << "GAEC loop with queue size: " << this->edgePriorityQueue.size() << endl;
        pair<float, int> biggestCostEdge = *this->edgePriorityQueue.begin();
        this->edgePriorityQueue.erase(this->edgePriorityQueue.begin());
        float biggestCost = biggestCostEdge.first;
        if (biggestCost < 0)
        {
            // cout << "GAEC loop breaks because cost < 0" << endl;
            break;
        }

        pair<int, int> decodedEdge = utils::elegantUnpair(biggestCostEdge.second);
        int edgeVertex1 = decodedEdge.first;
        vector<pair<float, int>> edgeVertex1Neighbors = adjacencies.at(edgeVertex1);
        int edgeVertex2 = decodedEdge.second;
        vector<pair<float, int>> edgeVertex2Neighbors = adjacencies.at(edgeVertex2);
        set<pair<float, int>> edgeVerticesNeighbors; // can have duplicated neighbors with # costs, because they might be shared between v1 & v2
        edgeVerticesNeighbors.insert(edgeVertex1Neighbors.cbegin(), edgeVertex1Neighbors.cend());
        edgeVerticesNeighbors.insert(edgeVertex2Neighbors.cbegin(), edgeVertex2Neighbors.cend());

        // join v1 & v2, so change all children with v2 as parent to have v1 as parent
        for (const auto &[child, parent] : this->unionFindParentMap)
        {
            if (parent == edgeVertex2)
            {
                this->unionFindParentMap[child] = edgeVertex1;
            }
        }

        unordered_set<int> seenNeighbors;
        // go through v1's neighbors
        for (auto &neighbor : edgeVerticesNeighbors)
        {
            // update the entry inside this->edgePriorityQueue & edgeCostMap
            float neighborCost = neighbor.first;
            int neighborVertex = neighbor.second;
            if (neighborVertex == edgeVertex1 || neighborVertex == edgeVertex2)
                continue;

            // ---- delete v1-neighbor from priority queue (IF ANY - this applies to the case where v1 & v2 have a common neighbor but w # costs)
            int encodedEdgeV1 = this->encodeEdge(edgeVertex1, neighborVertex);
            pair<float, int> edgePriorityQueueEntryV1 = make_pair(neighborCost, encodedEdgeV1);
            this->edgePriorityQueue.erase(edgePriorityQueueEntryV1);
            // ---- delete v2-neighbor from priority queue (IF ANY)
            int encodedEdgeV2 = this->encodeEdge(edgeVertex2, neighborVertex);
            pair<float, int> edgePriorityQueueEntryV2 = make_pair(neighborCost, encodedEdgeV2);
            this->edgePriorityQueue.erase(edgePriorityQueueEntryV2);

            if (seenNeighbors.contains(neighborVertex))
            {
                // this neighbor was processed before (because it is a shared neighbor between v1 & v2 so it probably appears twice)
                // we have deleted from priority queue (as above)
                continue;
            }
            seenNeighbors.insert(neighborVertex);

            // ---- calculate new cost for v1-neighbor
            int v1NeighborCode = this->encodeEdge(edgeVertex1, neighborVertex);
            int v2NeighborCode = this->encodeEdge(edgeVertex2, neighborVertex);
            float costV1Neighbor = this->edgeCostMap.contains(v1NeighborCode) ? this->edgeCostMap.at(v1NeighborCode) : 0;
            float costV2Neighbor = this->edgeCostMap.contains(v2NeighborCode) ? this->edgeCostMap.at(v2NeighborCode) : 0;
            float newCost = costV1Neighbor + costV2Neighbor;

            // ---- add v1-neighbor with new cost back into priority queue
            pair<float, int> newPriorityQueueEntry = make_pair(newCost, v1NeighborCode);
            this->edgePriorityQueue.insert(newPriorityQueueEntry);
            // ---- add v1-neighbor with new cost back into edgeCostMap (IF IT ALREADY EXISTS, else add to map)
            this->edgeCostMap[encodedEdgeV1] = newCost;
        }

        // update all edges in adjacencies with v1's parent to have new cost
        // & update all edges in adjacencies with v2's parent to be with v1's parent & to have new cost
        // ---- update edges with v1's neighbors with new costs
        seenNeighbors.clear();
        vector<pair<float, int>> newEdgeVertex1Neighbors;
        for (pair<float, int> v1Neighbor : edgeVertex1Neighbors)
        {
            int neighbor = v1Neighbor.second;
            if (neighbor == edgeVertex2)
                continue; // if adjacency = v2, skip

            int encodedEdge = this->encodeEdge(neighbor, edgeVertex1);
            float newCost = this->edgeCostMap.at(encodedEdge);
            newEdgeVertex1Neighbors.push_back(make_pair(newCost, neighbor));

            // update neighbors of neighbor where v1 is there!
            vector<pair<float, int>> newNeighborsNeighbors;
            vector<pair<float, int>> neighborsNeighbors = adjacencies.at(neighbor);
            for (pair<float, int> neighborsNeighbor : neighborsNeighbors)
            {
                if (neighborsNeighbor.second == edgeVertex1)
                {
                    pair<float, int> newNeighborsNeighbor = make_pair(newCost, edgeVertex1);
                    newNeighborsNeighbors.push_back(newNeighborsNeighbor);
                }
                else if (neighborsNeighbor.second == edgeVertex2)
                {
                    // we don't want neighbor-vertex2 to appear in adjacencies!
                    continue;
                }
                else
                {
                    newNeighborsNeighbors.push_back(neighborsNeighbor);
                }
            }
            adjacencies[neighbor] = newNeighborsNeighbors;
            seenNeighbors.insert(neighbor);
        }
        // ---- replace edges with v2's neighbors to be v1-neighbor and with new costs
        for (pair<float, int> v2Neighbor : edgeVertex2Neighbors)
        {
            int neighbor = v2Neighbor.second;
            if (neighbor == edgeVertex1)
                continue; // if adjacency = v1, skip

            // edgeVertex1, because v1 is now v2's parent
            int encodedEdge = this->encodeEdge(neighbor, edgeVertex1);
            float newCost = this->edgeCostMap.at(encodedEdge);

            if (!seenNeighbors.contains(neighbor))
            { // in case v1 already connected to neighbor (neighbor is shared between v1 & v2, so it was processed
                // above

                newEdgeVertex1Neighbors.push_back(make_pair(newCost, neighbor));
                // update neighbors of neighbor where v2 is there!
                // specifically, replace v2 with v1, and use new cost
                vector<pair<float, int>> newNeighborsNeighbors;
                vector<pair<float, int>> neighborsNeighbors = adjacencies.at(neighbor);
                for (pair<float, int> neighborsNeighbor : neighborsNeighbors)
                {
                    // if other vertex is v2
                    if (neighborsNeighbor.second == edgeVertex2)
                    {
                        // then we use v1 (!!!) & new cost
                        pair<float, int> newNeighborsNeighbor = make_pair(newCost, edgeVertex1);
                        newNeighborsNeighbors.push_back(newNeighborsNeighbor);
                    }
                    else
                    {
                        newNeighborsNeighbors.push_back(neighborsNeighbor);
                    }
                }
                adjacencies[neighbor] = newNeighborsNeighbors;
            }
        }
        // ---- remove v2 from adjacencies
        adjacencies.erase(edgeVertex2);
        // ---- update v1's neighbors & edge costs
        adjacencies[edgeVertex1] = newEdgeVertex1Neighbors;
    }
}

void Mesh::KernighanLin()
{
    //
}

vector<int> Mesh::getNodesUnderSameParentFromUnionFind(int child)
{
    int parent = this->unionFindParentMap.at(child);
    vector<int> children;
    for (const auto &[mapChild, mapParent] : this->unionFindParentMap)
    {
        if (mapParent == parent && mapChild != child)
        {
            // child passed in is the root node. We don't want the root node in our output
            children.push_back(mapChild);
        }
    }
    return children;
}

void addedge(unordered_map<int, vector<std::pair<float, int>>> *adjacencymap, int i, int j, float v)
{
    if (adjacencymap->contains(i))
    {
        adjacencymap->at(i).push_back(std::make_pair(v, j));
    }
    else
    {
        vector<std::pair<float, int>> theset = vector<std::pair<float, int>>();
        theset.push_back(std::make_pair(v, j));
        adjacencymap->emplace(i, theset);
    }

    if (adjacencymap->contains(j))
    {
        adjacencymap->at(j).push_back(std::make_pair(v, i));
    }
    else
    {
        vector<std::pair<float, int>> theset = vector<std::pair<float, int>>();
        theset.push_back(std::make_pair(v, i));
        adjacencymap->emplace(j, theset);
    }
}

bool doTrianglesShareEdge(Vector3i t1, Vector3i t2)
{
    int shared = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (t1[i] == t2[j])
            {
                shared = shared + 1;
            }
        }
    }
    if (shared == 2)
    {
        return true;
    }
    else if (shared > 2)
    {
        std::cout << "warning- two triangles share more than one edge in common" << std::endl;
        return true;
    }
    return false;
}

bool Mesh::areVerticesContinuous(Vertex *v1, Vertex *v2)
{
    if (v1->lineIndex != v2->lineIndex)
        return false;

    vector<int> line = this->_lines[v1->lineIndex];
    for (int i = 0; i < line.size(); i++)
    {
        int vertexIndex = line[i];
        Vertex *curVertex = this->_vertices[vertexIndex];
        if (curVertex == v1)
        {
            // check if v2 is the next one in line
            if (i < line.size() && this->_vertices[line[i + 1]] == v2)
            {
                return true;
            }
        }
        else if (curVertex == v2)
        {
            // check if v2 is the next one in line
            if (i < line.size() && this->_vertices[line[i + 1]] == v1)
            {
                return true;
            }
        }
    }
    return false;
}

unordered_map<int, vector<pair<float, int>>> Mesh::makeGraph(vector<Vector3i> trianglepatch, unordered_map<int, unordered_set<int>> incompatibles)
{
    int numtriangles = trianglepatch.size();
    unordered_map<int, vector<pair<float, int>>> adjacencies;

    unordered_map<int, int> indicestohashes = unordered_map<int, int>();
    for (int i = 0; i < numtriangles; i++)
    {
        // make a node for that triangle
        // Note: I don't think we even need a node structure.
        indicestohashes.emplace(i, tohash(trianglepatch[i], _vertices.size()));
    }
    // Also we need an output node. That will be below vvv
    indicestohashes.emplace(-1, -1);
    // Now look at...like.... ever single pair of triangles. F
    for (int i = 0; i < numtriangles; i++)
    {
        if (!incompatibles.contains(i))
        {
            unordered_set<int> emptySet;
            incompatibles[i] = emptySet;
        }

        for (int j = i + 1; j < numtriangles; j++)
        {
            if (!incompatibles.contains(j))
            {
                unordered_set<int> emptySet;
                incompatibles[j] = emptySet;
            }

            bool mutuallyincompatible = incompatibles.at(i).contains(j) || incompatibles.at(j).contains(i);
            // If they are mutually incompatible make an arc and give it a negative weight
            if (mutuallyincompatible)
            {
                addedge(&adjacencies, i, j, -30.f);
            }
            else
            {
                bool commonedge = doTrianglesShareEdge(trianglepatch[i], trianglepatch[j]);
                // else: If they are not mutually incompatible but share a common edge give it a weight of 1
                if (commonedge)
                {
                    addedge(&adjacencies, i, j, 1.f);
                }
            }
        }
    }
    // Make an edge between every triangle and the output node as well.
    // Last node's index = numtriangles
    for (int i = 0; i < numtriangles; i++)
    {
        // Assign the weight based on the matching score
        float matchingweight = 0.f;

        // check which 2 of them are on the same stroke
        Vertex *v1 = this->_vertices[trianglepatch[i][0]];
        Vertex *v2 = this->_vertices[trianglepatch[i][1]];
        Vertex *v3 = this->_vertices[trianglepatch[i][2]];
        // first, check if all 3 are on the same stroke:
        if (this->areVerticesContinuous(v1, v2))
        {
            // v3 is the other vertex
            // does not matter if I check left/right with v1 or v2
            Vector3f v1v3 = (v3->position - v1->position).normalized();
            bool leftside = v1->binormal.dot(v1v3) < 0;
            matchingweight = this->vertexVertexScore(v1, v3, leftside) + this->vertexVertexScore(v2, v3, leftside);
        }
        else if (this->areVerticesContinuous(v3, v2))
        {
            // v1 is the other vertex
            Vector3f v3v1 = (v1->position - v3->position).normalized();
            bool leftside = v3->binormal.dot(v3v1) < 0;
            matchingweight = this->vertexVertexScore(v3, v1, leftside) + this->vertexVertexScore(v2, v1, leftside);
        }
        else if (this->areVerticesContinuous(v1, v3))
        {
            // v2 is the other vertex
            Vector3f v1v2 = (v2->position - v1->position).normalized();
            bool leftside = v1->binormal.dot(v1v2) < 0;
            matchingweight = this->vertexVertexScore(v1, v2, leftside) + this->vertexVertexScore(v3, v2, leftside);
        }
        else
        {
            matchingweight = -30.f;
            //            cerr << "Triangle " << trianglepatch[i][0] << "-" << trianglepatch[i][1] << "-" << trianglepatch[i][2] << " does not have any continuous, same-line pair of vertices";
        }
        //        int v1Index = this->getVertexIndex(v1);
        //        int v2Index = this->getVertexIndex(v2);
        int v1Index = trianglepatch[i][0];
        int v2Index = trianglepatch[i][1];
        int v3Index = trianglepatch[i][2];
        matchingweight += this->outputTrianglesEdges.contains(this->encodeEdge(v1Index, v2Index));
        matchingweight += this->outputTrianglesEdges.contains(this->encodeEdge(v3Index, v2Index));
        matchingweight += this->outputTrianglesEdges.contains(this->encodeEdge(v3Index, v1Index));

        addedge(&adjacencies, i, numtriangles, matchingweight);
    }
    return adjacencies;
}

vector<pair<map<int,int>,vector<pair<int,int>>>> clumpPairs(vector<pair<int,int>> boundarypairs){
    vector<pair<map<int,int>,vector<pair<int,int>>>> lineclumps = vector<pair<map<int,int>,vector<pair<int,int>>>>();

    //Go through the vector of boundary pairs and put everything into a vector of pairs
    for(int i = 0; i < boundarypairs.size(); i++){
        pair<int,int> thepair = boundarypairs[i];

        //Go through each clump. If it contains the pair, add it.
        bool wasfound = false;

        for(int j = 0; j < lineclumps.size(); j++){
            bool immediatelyfound = false;
            //Look for the pair in each of the lineclump maps. If you found it then you should increase the map value. Also add the pair to the right side

            //So the issue here is that we're adding stuff before we finish incrementing.
            //Incrementing should be one step, then adding stuff should be another step

            if(lineclumps[j].first.contains(thepair.first)){
                //If the clump contains the first element of the pair, increase its count
                lineclumps[j].first.at(thepair.first) += 1;
                wasfound = true;
                immediatelyfound = true;
                //Also add the second one
                if(lineclumps[j].first.contains(thepair.second)){
                    //If the clump contains the second element of the pair, add the pair to the map. Increase the pair's value
                    lineclumps[j].first.at(thepair.second) += 1;
                }
            }
            else if(lineclumps[j].first.contains(thepair.second)){
                //If the clump contains the second element of the pair, add the pair to the map. Increase the pair's value
                lineclumps[j].first.at(thepair.second) += 1;
                wasfound = true;
                immediatelyfound = true;
                //Also add the first one
                if(lineclumps[j].first.contains(thepair.first)){
                    //If the clump contains the second element of the pair, add the pair to the map. Increase the pair's value
                    lineclumps[j].first.at(thepair.first) += 1;
                }
            }

            if(lineclumps[j].first.contains(thepair.first)){
                if(!lineclumps[j].first.contains(thepair.second)){
                    lineclumps[j].first.emplace(thepair.second, 1);
                    // wasfound = true;
                    // immediatelyfound = true;
                }
            }
            if(lineclumps[j].first.contains(thepair.second)){
                if(!lineclumps[j].first.contains(thepair.first)){
                    lineclumps[j].first.emplace(thepair.first, 1);
                    // wasfound = true;
                    // immediatelyfound = true;
                }
            }
            //If it was just now found, add it. Otherwise, don't
            if(immediatelyfound){
                lineclumps[j].second.push_back(thepair);
            }
        }

        //If it wasn't found in any of the clumps, add it to a new clump
        if(!wasfound){
            map<int,int> newmap = map<int,int>();
            newmap.emplace(thepair.first, 1);
            newmap.emplace(thepair.second, 1);

            vector<pair<int,int>> newpairlist = vector<pair<int,int>>();
            newpairlist.push_back(thepair);

            pair<map<int,int>,vector<pair<int,int>>> newclump = make_pair(newmap,newpairlist);
            lineclumps.push_back(newclump);
        }
    }
    return lineclumps;
}

void Mesh::computeBoundaries(){
    _boundaries.clear();

    //TODO: remember what the heck you were cooking with wasfound and immediatelyfound

    bool debugprint = false;

    map<pair<int,int>,int> edgecounts;
    //Accumulate a map that goes from int pairs to int counts.
    for(int i = 0; i < _faces.size(); i++){
        Vector3i currtriangle = sortvec(_faces[i]);
        //Get each edge
        pair<int,int> e01 = make_pair(currtriangle[0],currtriangle[1]);
        pair<int,int> e12 = make_pair(currtriangle[1],currtriangle[2]);
        pair<int,int> e02 = make_pair(currtriangle[0],currtriangle[2]);
        if(edgecounts.contains(e01)){
            edgecounts.at(e01) = edgecounts.at(e01) + 1;
        }
        else{
            edgecounts.emplace(e01, 1);
        }

        if(edgecounts.contains(e12)){
            edgecounts.at(e12) = edgecounts.at(e12) + 1;
        }
        else{
            edgecounts.emplace(e12, 1);
        }

        if(edgecounts.contains(e02)){
            edgecounts.at(e02) = edgecounts.at(e02) + 1;
        }
        else{
            edgecounts.emplace(e02, 1);
        }
    }

    //What now? Iterate through all of edgecounts and only keep the keys(pairs) that have a value of 1. Store them in a vector.
    std::vector<pair<int,int>> boundarypairs = std::vector<pair<int,int>>();
    int unchanged  = 0;
    for (auto const& [key, val] : edgecounts){
        if(val == 1){
            boundarypairs.push_back(key);
            // _vertices[key.first]->position = _vertices[key.first]->position + 0.05*_vertices[key.first]->normal;
            // _vertices[key.second]->position = _vertices[key.second]->position + 0.05*_vertices[key.second]->normal;
        }
        else{
            unchanged += 1;
        }
    }
    std::cout << "Total number of boundary pairs: " + std::to_string(boundarypairs.size()) << std::endl;
    //Now what? Try displacing every single set of boundary pairs. Like just scale them up or something.
    std::cout << "Total number of non-boundary pairs: " + std::to_string(unchanged) << std::endl;

    //What do we want to do now? create a map of counts for each index hit.
    map<int,int> generalcounts;
    for(int i = 0; i < boundarypairs.size(); i++){
        pair<int,int> thepair = boundarypairs[i];
        if(generalcounts.contains(thepair.first)){
            generalcounts.at(thepair.first) += 1;
        }
        else{
            generalcounts.emplace(thepair.first, 1);
        }
        if(generalcounts.contains(thepair.second)){
            generalcounts.at(thepair.second) += 1;
        }
        else{
            generalcounts.emplace(thepair.second, 1);
        }
    }

    vector<pair<int,int>> filteredpairs;
    //Once the map is full, go through each pair and make sure that the counts of the first and second indices are less than or equal to 2. Add the valid
    //ones to a list of filtered pairs.
    for(int i = 0; i < boundarypairs.size(); i++){
        pair<int,int> thepair = boundarypairs[i];
        if((generalcounts.at(thepair.first) <= 2) && (generalcounts.at(thepair.second) <= 2)){
            filteredpairs.push_back(thepair);
        }
    }


    //Sick! We now have all boundary pairs. We now need to convert these pairs into polylines.
    vector<pair<map<int,int>,vector<pair<int,int>>>> lineclumps = clumpPairs(filteredpairs);

    // Now what? lineclumps should be full of stuff by now

    vector<pair<pair<bool,int>,vector<pair<int,int>>>> line_cycles = vector<pair<pair<bool,int>,vector<pair<int,int>>>>();

    for(int i = 0; i < lineclumps.size(); i++){
        vector<pair<int,int>> thepairs = lineclumps[i].second;
        int numones = 0;
        string mapstring = "";
        string pairstring = "";
        bool brokeasf = false;
        int startindex = -1;

        for (auto const& [key, val] : lineclumps[i].first){
            // mapstring = mapstring + "(" + to_string(key) + ": " + to_string(val) + ")";
            if(val == 1){
                numones += 1;
                startindex = key;
            }
            if(val > 2){
                brokeasf = true;
            }
        }
        // for(int k = 0; k < thepairs.size(); k++){
        //     pairstring = pairstring + "(" + to_string(thepairs[k].first) + ", " + to_string(thepairs[k].second) + ")";
        // }
        // Either it isn't brokeasf and has 2 numones
        //Or it is brokeasf and has 0 numones
        // cout << "Clump " + to_string(i) + " contains the following pairs: " + pairstring << endl;
        // cout << "Clump " + to_string(i) + " contains the following map: " + mapstring << endl;


        if((!brokeasf) && ((numones == 2) || numones == 0)){
            line_cycles.push_back(make_pair(make_pair((numones == 2),startindex),thepairs));
        }
    }
    //Line cycles is now full of vectors: bool followed by pairs. If it is a line then true. If it is a cycle then false.
    //Now what? go through each line cycle, arbitrarily pick an end, and unspool it

    //But first let's print each line cycle.
    if(debugprint){
        for(int i = 0; i < line_cycles.size(); i++){
            string linestring = "";
            string cycletype;
            pair<pair<bool,int>,vector<pair<int,int>>> currcycle = line_cycles[i];
            vector<pair<int,int>> pairs = currcycle.second;
            for(int j = 0; j < pairs.size() ; j++){
                linestring = linestring + "(" + to_string(pairs[j].first) + ", " + to_string(pairs[j].second) + ") ";
            }
            if(currcycle.first.first){
                cycletype = "line";
            }
            else{
                cycletype = "cycle";
            }
            cout << "entry " + to_string(i) + " is a " + cycletype + ". Starts at " + to_string(currcycle.first.second) + " and contains: " + linestring << endl;
        }
    }

    //Sick. Now let's shove all of this stuff into another function to turn these into proper lines/cycles.
    //What do we want at the end? A vector of pair<bool, vector<int>>. Push directly into _boundaries
    for(int i = 0; i < line_cycles.size(); i++){
        pair<pair<bool,int>,vector<pair<int,int>>> currcycle = line_cycles[i];
        //Identify the starting pair if it is a line or a cycle.
        vector<pair<int,int>> remainingpairs = currcycle.second;

        vector<int> sorted_indices = vector<int>();
        int taildex;
        if(currcycle.first.first){
            sorted_indices.push_back(currcycle.first.second);
            taildex = currcycle.first.second;
        }
        else{
            sorted_indices.push_back(remainingpairs[0].first);
            taildex = remainingpairs[0].first;
        }

        //DONE I have a feeling we can streamline the above code so it doesn't have to happen twice. Just set taildex and enter the loop.
        //Ok. We have now erased an element from remainingpairs. Should be good to iterate.

        //Do this while there are still pairs left

        // cout << "Entering while loop with the following pairs: " << endl;
        // if(debugprint){
        //     string linestring = "";
        //     for(int j = 0; j < remainingpairs.size(); j++){
        //         linestring = linestring + "(" + to_string(remainingpairs[j].first) + ", " + to_string(remainingpairs[j].second) + ") ";
        //     }
        //     cout << "Cycleline contains: " + linestring << endl;
        // }
        int loopnum = 0;
        while(remainingpairs.size() > 0){
            loopnum +=1;
            // cout << "Loop number: " + to_string(loopnum) + " Remaining pairs: " + to_string(remainingpairs.size()) << endl;
            //Find the pair containing taildex
            for(int j = 0; j < remainingpairs.size(); j++){
                //Does the current pair contain taildex? If so, add its last element and update taildex. Then remove it.
                if(remainingpairs[j].first == taildex){
                    sorted_indices.push_back(remainingpairs[j].second);
                    taildex = remainingpairs[j].second;
                    remainingpairs.erase(remainingpairs.begin() + j);
                    break;
                }
                else if(remainingpairs[j].second == taildex){
                    sorted_indices.push_back(remainingpairs[j].first);
                    taildex = remainingpairs[j].first;
                    remainingpairs.erase(remainingpairs.begin() + j);
                    break;
                }
            }
        }
        //Now what? sorted_indices should be fully populated by now. Add it to _boundaries.
        _boundaries.push_back(make_pair(currcycle.first.first, sorted_indices));
    }
    //Boundaries should now be fully populated. Print all boundaries.
    if(debugprint){
        for(int i = 0; i < _boundaries.size(); i++){
            string intstring = "";
            for(int j = 0; j < _boundaries[i].second.size(); j++){
                intstring = intstring + to_string(_boundaries[i].second[j]) + ", ";
            }
            string linetype;
            if(_boundaries[i].first){
                linetype = "line";
            }
            else{
                linetype = "cycle";
            }

            cout << linetype + " " + to_string(i) + " contains the following indices: [" + intstring + "]" << endl;
        }
    }

    cout << "Boundary computation finished: Located " + to_string(_boundaries.size()) + " boundaries." << endl;
}

//Now what? Time to actually smooth the boundaries
void Mesh::smoothBoundaries(){
    //Chloe supposedly made a map from edges to faces. We will need that here
    //Go through each boundary string and find the new vertex positions of each vertex
    //After that, if it is a cycle, smooth the end vertex.
    //Note: _boundaries lists the start index at the end as well if it is a cycle. Just remember that

    for(int i = 0; i < _boundaries.size(); i++){
        pair<bool,vector<int>> currboundary = _boundaries[i];
        //If currboundary's size is 2 then don't do anything

        map<int,Vector3f> newpositions = map<int,Vector3f>();

        if(currboundary.second.size() > 2){
            for(int j = 1; j < currboundary.second.size()-1; j++){
                //Start at the second entry and finish at the second to last one.
                Vector3f pj = _vertices[currboundary.second[j]]->position;
                Vector3f pjminus = _vertices[currboundary.second[j-1]]->position;
                Vector3f pjplus = _vertices[currboundary.second[j+1]]->position;
                Vector3f new_position = 0.5*pj + 0.25*(pjminus + pjplus);

                newpositions.emplace(currboundary.second[j], new_position);
            }
            //Now what? If it is a cycle then you should do the first vertex too. It's previous is the second to last

            if(!currboundary.first){
                Vector3f pj = _vertices[currboundary.second[0]]->position;
                Vector3f pjminus = _vertices[currboundary.second[currboundary.second.size()-2]]->position;
                Vector3f pjplus = _vertices[currboundary.second[1]]->position;
                Vector3f new_position = 0.5*pj + 0.25*(pjminus + pjplus);

                //TODO: Do a check here to see if changing this would actually break anything.
                //First we need that map that goes from edges to triangles
                // map<pair<int,int>,vector<Vector3i>> edges_to_triangles;

                Vector3i oldtriminus = edges_to_triangles.at(sortpair(currboundary.second[0], currboundary.second[currboundary.second.size()-2]))[0];
                Vector3i oldtriplus = edges_to_triangles.at(sortpair(currboundary.second[0], currboundary.second[1]))[0];

                Vector3f oldminusnorm = getnorm(_vertices[oldtriminus[0]]->position,_vertices[oldtriminus[1]]->position,_vertices[oldtriminus[2]]->position);
                Vector3f oldplusnorm = getnorm(_vertices[oldtriplus[0]]->position,_vertices[oldtriplus[1]]->position,_vertices[oldtriplus[2]]->position);

                //Get the old triangles and their normals

                // If it doesn't break use it
                //If if does break use pj
                newpositions.emplace(currboundary.second[0], new_position);
                //Now what? Make triangles out of these new positions

                //We have to make a vector of vector3fs from the old triangle but swapping out the index that wasn't the first two with the old position.
                //Is there any better way to do this? I don't think so because the order has to be the same
                vector<Vector3f> newminuspositions;
                vector<Vector3f> newpluspositions;
                for(int k = 0; k < 3; k++){
                    //Check if it's equal to uhhhhhhh currboundarysecond or currboundary second size minus 2
                    if((oldtriminus[k] == currboundary.second[0]) || (oldtriminus[k] ==  currboundary.second[currboundary.second.size()-2])){
                        //If it's equal then use the new one.
                        newminuspositions.push_back(newpositions.at(oldtriminus[k]));
                    }
                    else{
                        //Otherwise use the old position.
                        newminuspositions.push_back(_vertices[oldtriminus[k]]->position);
                    }

                    if((oldtriplus[k] == currboundary.second[0]) || (oldtriplus[k] ==  currboundary.second[1])){
                        //If it's equal then use the new one.
                        newpluspositions.push_back(newpositions.at(oldtriplus[k]));
                    }
                    else{
                        //Otherwise use the old position.
                        newpluspositions.push_back(_vertices[oldtriplus[k]]->position);
                    }
                }

                Vector3f newminusnorm = getnorm(newminuspositions[0],newminuspositions[1],newminuspositions[2]);
                Vector3f newplusnorm = getnorm(newpluspositions[0], newpluspositions[1], newpluspositions[2]);

                //Now we just need to take the difference in norms! See if it changes much
                if((acos(oldminusnorm.dot(newminusnorm)) > M_PI*0.24) || (acos(oldplusnorm.dot(newplusnorm)) > M_PI*0.24)){
                    //Then don't use the new position. Keep the old position for the jth thing
                    // cout << "It is too bent" << endl;
                    newpositions.emplace(currboundary.second[0], _vertices[currboundary.second[0]]->position);
                }
                else{
                    // cout << "It is ok " << endl;
                }
            }

            //Sweet. By now, newpositions should all be filled. Update vertex positions
            for (auto const& [key, val] : newpositions){
                //Update the keyth position in vertices with val
                _vertices[key]->position = val;
            }
        }
    }
}

// re-calculate boundary binormals that point out from the surface
// process each boundary in vector<pair<bool,vector<int>>> _boundaries, first recalculating tangents
void Mesh::getNewBoundaryTangentsAndBinormals() {
    // If the vector represents a line, the bool is true. If the vector represents a cycle, the bool is false
    // For cycles, the start index is listed at the end as well
    for (auto & boundary_line : _boundaries) {
        for (int i = 0; i < boundary_line.second.size(); i++) {
            int boundary_vertex = boundary_line.second[i];
            vector<int> line = boundary_line.second;
            int n = line.size();
            if (boundary_line.first) { // =========== it's a line
                // get new tangents
                // loop through all vertices on the line
                Vector3f first_segment = (_vertices[line[1]]->position - _vertices[line[0]]->position).normalized(); // tangent of the first vertex is just the line segment direction
                // ===== changed first tangent: need to project it onto the orthogonal direction of the normal!!
                Vector3f cur_normal = _vertices[line[0]]->normal;
                Vector3f first_segment_parallel = first_segment.dot(cur_normal) / (cur_normal.norm() * cur_normal.norm()) * cur_normal;
                Vector3f first_tangent = (first_segment - first_segment_parallel).normalized();

                _vertices[line[0]]->boundary_tangent = first_tangent;
                _vertices[line[0]]->boundary_binormal = first_tangent.cross(_vertices[line[0]]->normal).normalized();
                // check that the binormal is pointing outwards: get a random triangle at vertex line[0]
                // unordered_map<int, vector<Vector3i>> vertexToTriangles;
                Vector3i tri0 = vertexToTriangles.at(line[0])[0];
                if (tri0[0] != line[0]) {
                    // use the vector tri[0] - line[0]: binormals should have negative dot product with this vector
                    if (_vertices[line[0]]->boundary_binormal.dot(_vertices[tri0[0]]->position - _vertices[line[0]]->position) > 0) _vertices[line[0]]->boundary_binormal = - _vertices[line[0]]->boundary_binormal;
                }else {
                    // use the vector tri[1] - line[0]: binormals should have negative dot product with this vector
                    if (_vertices[line[0]]->boundary_binormal.dot(_vertices[tri0[1]]->position - _vertices[line[0]]->position) > 0) _vertices[line[0]]->boundary_binormal = - _vertices[line[0]]->boundary_binormal;
                }

                for (int i = 1; i < n - 1; i++) {
                    Vector3f curA = _vertices[line[i - 1]]->position;
                    Vector3f curB = _vertices[line[i]]->position;
                    Vector3f curC = _vertices[line[i + 1]]->position;
                    Vector3f AC = curC - curA;
                    Vector3f B_normal = _vertices[line[i]]->normal;
                    Vector3f AC_parallel = AC.dot(B_normal) / (B_normal.norm() * B_normal.norm()) * B_normal; // AC projected onto the direction of normal

                    Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
                    _vertices[line[i]]->boundary_tangent = cur_tangent;
                    _vertices[line[i]]->boundary_binormal = cur_tangent.cross(_vertices[line[i]]->normal).normalized();
                    // check that the binormal is pointing outwards: get a random triangle at vertex B: line[i]
                    // unordered_map<int, vector<Vector3i>> vertexToTriangles;
                    Vector3i tri = vertexToTriangles.at(line[i])[0];
                    if (tri[0] != line[i]) {
                        // use the vector tri[0] - line[0]: binormals should have negative dot product with this vector
                        if (_vertices[line[i]]->boundary_binormal.dot(_vertices[tri[0]]->position - _vertices[line[i]]->position) > 0) _vertices[line[i]]->boundary_binormal = - _vertices[line[i]]->boundary_binormal;
                    }else {
                        // use the vector tri[1] - line[0]: binormals should have negative dot product with this vector
                        if (_vertices[line[i]]->boundary_binormal.dot(_vertices[tri[1]]->position - _vertices[line[i]]->position) > 0) _vertices[line[i]]->boundary_binormal = - _vertices[line[i]]->boundary_binormal;
                    }
                }
                Vector3f last_segment = (_vertices[line[n - 1]]->position - _vertices[line[n - 2]]->position).normalized(); // tangent of the last vertex is just the line segment direction
                // ===== changed last tangent: need to project it onto the orthogonal direction of the normal!!
                cur_normal = _vertices[line[n-1]]->normal;
                Vector3f last_segment_parallel = last_segment.dot(cur_normal) / (cur_normal.norm() * cur_normal.norm()) * cur_normal;
                Vector3f last_tangent = (last_segment - last_segment_parallel).normalized();
                _vertices[line[n - 1]]->boundary_tangent = last_tangent;
                _vertices[line[n - 1]]->boundary_binormal = last_tangent.cross(_vertices[line[n - 1]]->normal).normalized();
                // check that the binormal is pointing outwards: get a random triangle at vertex B: line[n-1]
                // unordered_map<int, vector<Vector3i>> vertexToTriangles;
                Vector3i tri = vertexToTriangles.at(line[n-1])[0];
                if (tri[0] != line[n-1]) {
                    // use the vector tri[0] - line[0]: binormals should have negative dot product with this vector
                    if (_vertices[line[n-1]]->boundary_binormal.dot(_vertices[tri[0]]->position - _vertices[line[n-1]]->position) > 0) _vertices[line[n-1]]->boundary_binormal = - _vertices[line[n-1]]->boundary_binormal;
                }else {
                    // use the vector tri[1] - line[0]: binormals should have negative dot product with this vector
                    if (_vertices[line[n-1]]->boundary_binormal.dot(_vertices[tri[1]]->position - _vertices[line[n-1]]->position) > 0) _vertices[line[n-1]]->boundary_binormal = - _vertices[line[n-1]]->boundary_binormal;
                }

            }else { // it's a cycle
                for (int i = 0; i < n; i++) {
                    Vector3f curA, curC;
                    Vector3f curB = _vertices[line[i]]->position;
                    if (i == 0 || i == n-1) { // repeated vertex in the cycle -- first and last
                        curA = _vertices[line[n - 2]]->position;
                        curC = _vertices[line[1]]->position;
                    }else {
                        curA = _vertices[line[i - 1]]->position;
                        curC = _vertices[line[i + 1]]->position;
                    }
                    Vector3f AC = curC - curA;
                    Vector3f B_normal = _vertices[line[i]]->normal;
                    Vector3f AC_parallel = AC.dot(B_normal) / (B_normal.norm() * B_normal.norm()) * B_normal; // AC projected onto the direction of normal

                    Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
                    _vertices[line[i]]->boundary_tangent = cur_tangent;
                    _vertices[line[i]]->boundary_binormal = cur_tangent.cross(_vertices[line[i]]->normal).normalized();
                    // check that the binormal is pointing outwards: get a random triangle at vertex B: line[i]
                    // unordered_map<int, vector<Vector3i>> vertexToTriangles;
                    Vector3i tri = vertexToTriangles.at(line[i])[0];
                    if (tri[0] != line[i]) {
                        // use the vector tri[0] - line[0]: binormals should have negative dot product with this vector
                        if (_vertices[line[i]]->boundary_binormal.dot(_vertices[tri[0]]->position - _vertices[line[i]]->position) > 0) _vertices[line[i]]->boundary_binormal = - _vertices[line[i]]->boundary_binormal;
                    }else {
                        // use the vector tri[1] - line[0]: binormals should have negative dot product with this vector
                        if (_vertices[line[i]]->boundary_binormal.dot(_vertices[tri[1]]->position - _vertices[line[i]]->position) > 0) _vertices[line[i]]->boundary_binormal = - _vertices[line[i]]->boundary_binormal;
                    }
                }


            }

        }
    }
}

// -------- PRIVATE ENDS -------------------------------------------------------------------------------
