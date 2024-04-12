#include "mesh.h"

#include <iostream>
#include <fstream>

#include <QFileInfo>
#include <QString>
#include <Eigen>

#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"
#include "util/ply_loader.h"
#include "util/obj_loader.h"

#include <fstream>

// -------- PUBLIC STARTS -------------------------------------------------------------------------------

void Mesh::loadFromFile(const std::string &inObjFilePath, const std::string &inPlyFilePath)
{
    // load obj file
    pair<vector<Vector3f>, vector<Vector2i>> verticesAndLineSegments = objLoader::loadFromFile(inObjFilePath);
    vector<Vector3f> vertices = verticesAndLineSegments.first;
    vector<Vector2i> lineSegments = verticesAndLineSegments.second;
    vector<vector<int>> lines = this->parseToPolyline(lineSegments);

    // load ply file
    vector<Vector3f> normals = plyLoader::loadFromFile(inPlyFilePath);

    // populate the _vertices vector
    vector<Vertex *> m_vertices;
    for (int i = 0; i < vertices.size(); i++)
    {
        Vertex *vertex = new Vertex(vertices[i], true, normals[i]);
        // vertex.tangent is set later in calculateTangents()
        m_vertices.push_back(vertex);
    }

    //    this->_vertices = vertices;
    this->_lines = lines;
    //    this->_vertexNormals = normals;
    this->_vertices = m_vertices;

    calculateTangents(vertices, normals);
}

void Mesh::calculateTangents(const vector<Vector3f> &vertices, const vector<Vector3f> &vertexNormals)
{
    // loop through all lines
    for (auto &line : _lines)
    {
        int n = line.size();
        // loop through all vertices on the line
        Vector3f first_tangent = vertices[line[1]] - vertices[line[0]]; // tangent of the first vertex is just the line segment direction
        _vertices[line[0]]->tangent = first_tangent;

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
        }
        Vector3f last_tangent = vertices[line[n - 1]] - vertices[line[n - 2]]; // tangent of the last vertex is just the line segment direction
        _vertices[line[n - 1]]->tangent = last_tangent;
    }
}

void Mesh::saveToFile(const string &outStrokeFilePath, const string &outMeshFilePath)
{
    ofstream outStrokeFile;
    outStrokeFile.open(outStrokeFilePath);

    ofstream outMeshFile;
    outMeshFile.open(outMeshFilePath);

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

void Mesh::debugSaveToFile(const string &outStrokeFilePath, const string &outMeshFilePath)
{
    // take in only 1st vertex and its candidates (left or right or both or none)
    vector<Vertex *> verticesForVisualization;
    int theOnlyBaseVertexWanted = 50;
    verticesForVisualization.push_back(this->_vertices[theOnlyBaseVertexWanted]);
    for (const auto &[baseVertexIndex, otherVertexIndices] : this->leftRestrictedMatchingCandidates)
    {
        //        if (theOnlyBaseVertexWanted == -1) {
        //            theOnlyBaseVertexWanted = baseVertexIndex;
        //            verticesForVisualization.push_back(this->_vertices[baseVertexIndex]);
        //        }
        if (baseVertexIndex != theOnlyBaseVertexWanted)
            continue;
        for (const auto &otherVertexIndex : otherVertexIndices)
        {
            verticesForVisualization.push_back(this->_vertices[otherVertexIndex]);
        }
        break;
    }
    for (const auto &[baseVertexIndex, otherVertexIndices] : this->rightRestrictedMatchingCandidates)
    {
        //        if (theOnlyBaseVertexWanted == -1) {
        //            theOnlyBaseVertexWanted = baseVertexIndex;
        //            verticesForVisualization.push_back(this->_vertices[baseVertexIndex]);
        //        }
        if (baseVertexIndex != theOnlyBaseVertexWanted)
            continue;
        for (const auto &otherVertexIndex : otherVertexIndices)
        {
            verticesForVisualization.push_back(this->_vertices[otherVertexIndex]);
        }
        break;
    }

    // strokes to visualize
    //    vector<vector<int>> strokesForVisualization;
    //    strokesForVisualization.push_back(this->_lines[1]);
    //    if (this->leftDominantStroke != -1) {
    //        strokesForVisualization.push_back(this->_lines[this->leftDominantStroke]);
    //    }
    //    if (this->rightDominantStroke != -1) {
    //        strokesForVisualization.push_back(this->_lines[this->rightDominantStroke]);
    //    }

    ofstream outStrokeFile;
    outStrokeFile.open(outStrokeFilePath);

    // Write vertices
    for (size_t i = 0; i < verticesForVisualization.size(); i++)
    {
        Vertex *v = verticesForVisualization[i];
        outStrokeFile << "v " << v->position[0] << " " << v->position[1] << " " << v->position[2] << endl;
    }

    // Write strokes
    //    for (size_t i = 0; i < strokesForVisualization.size(); i++)
    //    {
    //        const vector<int> &l = strokesForVisualization[i];
    //        for (size_t j = 0; j < l.size() - 1; j++)
    //        {
    //            outStrokeFile << "l " << (l[j] + 1) << " " << (l[j + 1] + 1) << endl;
    //        }
    //    }

    outStrokeFile.close();
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
}

void Mesh::cleanUp()
{
    for (int i = 0; i < this->_vertices.size(); i++)
    {
        delete this->_vertices[i];
    }
}

// -------- PUBLIC ENDS -------------------------------------------------------------------------------

// -------- PRIVATE STARTS -------------------------------------------------------------------------------

vector<vector<int>> Mesh::parseToPolyline(vector<Vector2i> connections)
{
    std::vector<std::vector<int>> polylines = std::vector<std::vector<int>>();
    int index = 0;
    while (index < connections.size())
    {
        vector<int> currentpoly = std::vector<int>();
        currentpoly.push_back(connections[index][0]);
        currentpoly.push_back(connections[index][1]);
        index = index + 1;
        while (connections[index][0] == currentpoly[currentpoly.size() - 1])
        {
            currentpoly.push_back(connections[index][1]);
            index = index + 1;
        }
        polylines.push_back(currentpoly);
    }
    return polylines;
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
    // This is specifically for a left match.
    Vector3f pc;
    if (leftside)
    {
        pc = p - strokewidth * pb;
    }
    else
    {
        pc = p + strokewidth * pb;
    }

    Vector3f ql = q - strokewidth * qb;
    Vector3f qr = q + strokewidth * qb;

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
    float finalscore = exp(-pow(da + dt + dln, 2) / (2.f * pow(sigma, 2.f)));
    return finalscore;
}

float Mesh::persistenceScore(Vertex *Pi, Vertex *Qi, Vertex *Pi_1, Vertex *Qi_1)
{
    Vector3f pi_1 = Pi_1->position;
    Vector3f qi_1 = Qi_1->position;
    Vector3f pi = Pi->position;
    Vector3f qi = Qi->position;
    float dp = ((pi_1 - pi) - (qi_1 - qi)).norm() + ((pi_1 - qi) - (qi_1 - pi)).norm() + ((pi_1 - qi_1) - (pi - qi)).norm();
    float finalscore = exp(-pow(dp, 2) / (2.f * pow(sigma, 2.f)));
    return finalscore;
}

// -------- PRIVATE ENDS -------------------------------------------------------------------------------
void Mesh::getRestrictedMatchingCandidates()
{
    assert(!this->_lines.empty());

    for (int i = 0; i < i < this->_lines.size(); i++)
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

                if (this->doTwoVerticesMatch(vertexIndex, vertexIndex2, true, true, i))
                {
                    if (!this->leftRestrictedMatchingCandidates.contains(vertexIndex))
                    {
                        this->leftRestrictedMatchingCandidates[vertexIndex] = unordered_set<int>();
                    }
                    unordered_set<int> leftMatches = this->leftRestrictedMatchingCandidates.at(vertexIndex);
                    leftMatches.insert(vertexIndex2);
                    this->leftRestrictedMatchingCandidates[vertexIndex] = leftMatches;
                }
                else if (this->doTwoVerticesMatch(vertexIndex, vertexIndex2, false, true, i))
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
    Vector3f baseBinormal = baseStrokeMidpointVertex->tangent.cross(baseStrokeMidpointVertex->normal);

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
        Vector3f baseToCurrVec = currStrokeMidpointVertex->position - baseStrokeMidpointVertex->position;

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
 * @brief Mesh::doTwoVerticesMatch
 * @param p
 * @param q
 * @param leftside
 * @param isOnSameStroke - whether p & q are on the same stroke
 * @param strokeIndex - -1 if isOnSameStroke == false, stroke index if isOnSameStroke == true
 * @return whether the vertices satisfy the condition
 */
bool Mesh::doTwoVerticesMatch(int pIndex, int qIndex, bool leftside, bool isOnSameStroke, int strokeIndex)
{
    Vertex *p = this->_vertices[pIndex];
    Vertex *q = this->_vertices[qIndex];
    Vector3f pq = q->position - p->position;
    Vector3f pBinormal = p->tangent.cross(p->normal).normalized();

    // condition 1
    float length = pq.norm();
    if (length > this->d_max)
    {
        return false;
    }

    // condition 2
    pq.normalize();
    float angle;
    if (leftside)
    {
        angle = acos(pq.dot(-pBinormal));
    }
    else
    {
        angle = acos(pq.dot(pBinormal));
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

    return true;
}

// can optimize, by "caching" results
int Mesh::calcNumberOfMatches(int baseStrokeIndex, int otherStrokeIndex, bool leftside)
{
    int baseCount = 0; // how many of base stroke's vertices got matched

    vector<int> baseStroke = this->_lines[baseStrokeIndex];
    vector<int> otherStroke = this->_lines[otherStrokeIndex];
    for (int i = 0; i < baseStroke.size(); i++)
    {
        int baseVertex = baseStroke[i];
        bool isEverMatched = false;

        for (int j = 0; j < otherStroke.size(); j++)
        {
            int otherVertex = otherStroke[j];
            bool baseOthermatched = this->doTwoVerticesMatch(baseVertex, otherVertex, leftside, false, -1);

            // TODO: IS THIS PART CORRECT?
            // !leftside is incorrect bc we don't know what side baseVertex is on w.r.t otherVertex
            // this is for p.9, section 5.2, restricted matching set subsection
            //            bool isEitherVertexEndVertex = (i == baseStroke.size() - 1 || i == 0) || (j == otherStroke.size() - 1 || j == 0);
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
            }
        }
        if (isEverMatched)
        {
            baseCount++;
        }
    }

    return baseCount;
}

// -------- PRIVATE ENDS -------------------------------------------------------------------------------
