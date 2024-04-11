#include "mesh.h"

#include <iostream>
#include <fstream>
#include <array>

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
    vector<Vertex*> m_vertices;
    for (int i = 0; i < vertices.size(); i++) {
        Vertex* vertex = new Vertex(vertices[i], true, normals[i]);
        // vertex.tangent is set later in calculateTangents()
        m_vertices.push_back(vertex);
    }

//    this->_vertices = vertices;
    this->_lines = lines;
//    this->_vertexNormals = normals;
    this->_vertices = m_vertices;

    calculateTangents(vertices, normals);
}

void Mesh::calculateTangents(const vector<Vector3f> &vertices, const vector<Vector3f> &vertexNormals) {
    // loop through all lines
    for (auto & line : _lines) {
        int n = line.size();
        // loop through all vertices on the line
        Vector3f first_tangent = vertices[line[1]] - vertices[line[0]]; // tangent of the first vertex is just the line segment direction
        _vertices[line[0]]->tangent = first_tangent;

        for (int i = 1; i < n - 1; i++) {
            Vector3f curA = vertices[line[i-1]];
            Vector3f curB = vertices[line[i]];
            Vector3f curC = vertices[line[i+1]];
            Vector3f AC = curC - curA;
            Vector3f B_normal = vertexNormals[line[i]];
            Vector3f AC_parallel =  AC.dot(B_normal) / (B_normal.norm()*B_normal.norm()) * B_normal; // AC projected onto the direction of normal

            Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
            _vertices[line[i]]->tangent = cur_tangent;

        }
        Vector3f last_tangent = vertices[line[n-1]] - vertices[line[n-2]]; // tangent of the last vertex is just the line segment direction
        _vertices[line[n-1]]->tangent = last_tangent;

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
        Vertex* v = _vertices[i];
        outMeshFile << "v " << v->position[0] << " " << v->position[1] << " " << v->position[2] << endl;
        outStrokeFile << "v " << v->position[0] << " " << v->position[1] << " " << v->position[2] << endl;

    }

    // Write vertex normals
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        Vertex* v = _vertices[i];
        outMeshFile << "vn " << v->normal[0] << " " << v->normal[1] << " " << v->normal[2] << endl;

    }

    // Write faces (MESH ONLY)
    for (size_t i = 0; i < _faces.size(); i++)
    {
        const Vector3i &f = _faces[i];
        outMeshFile << "f " << (f[0]+1) << " " << (f[1]+1) << " " << (f[2]+1) << endl;
    }

    // Write line segments (STROKE ONLY)
    for (size_t i = 0; i < _lines.size(); i++)
    {
        const vector<int> &l = _lines[i];
        for (size_t j = 0; j < l.size() - 1; j++) {
            outStrokeFile << "l " << (l[j]+1) << " " << (l[j+1]+1) << endl;
        }
    }

    outStrokeFile.close();
    outMeshFile.close();
}


// Preprocess the lines: check if the strokes have an abrupt direction change (angle of 45◦ or less between consecutive tangents)
// within 15% of overall stroke length from either end and remove the offending end-sections.
void Mesh::preprocessLines(){
    vector<vector<int>> new_lines;
    int itr = 0;
    // process each line at a time
    for (auto & line : _lines) {
        // first calculate line length -- sum of all segment lengths
        float total_length = 0;
        for (int i = 1; i < line.size(); i++) {
            // distance between vertex i and vertex i-1
            total_length += (this->_vertices[line[i]]->position - this->_vertices[line[i-1]]->position).norm();
        }

        float length_covered = 0;
        // forward direction: start to end until length_covered > 0.15 * total_length
        // calculate the tangents along the way and test if they differ by more than 45 degrees
        Vector3f prev_tangent = this->_vertices[line[1]]->position - this->_vertices[line[0]]->position; // tangent of the first vertex is just the line segment direction
        int cur_vert = 0; int next_vert = 1;
        int cut_pos_forward = 0; // the vertex at which we should cut the line (remove every vertex before this cut_pos)
        while (length_covered < 0.15 * total_length) {
            Vector3f curA = this->_vertices[line[cur_vert]]->position;
            Vector3f curB = this->_vertices[line[next_vert]]->position;
            Vector3f curC = this->_vertices[line[next_vert+1]]->position;
            Vector3f AC = curC - curA;
            Vector3f B_normal = this->_vertices[line[next_vert]]->normal;
            Vector3f AC_parallel =  AC.dot(B_normal) / (B_normal.norm()*B_normal.norm()) * B_normal; // AC projected onto the direction of normal

            Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
            if (acos(cur_tangent.dot(prev_tangent) / (cur_tangent.norm()*prev_tangent.norm())) > M_PI/4.0) {
                // remove everything before vertex B (starting from A)
                cut_pos_forward = next_vert;
            }
            // move on to the next segment
            prev_tangent = cur_tangent;
            length_covered += (curB - curA).norm();
            cur_vert++; next_vert++;
        }

        // backward direction: end to start
        int n = line.size();
        length_covered = 0;
        prev_tangent = this->_vertices[line[n-2]]->position - this->_vertices[line[n-1]]->position; // tangent of the first vertex is just the line segment direction
        cur_vert = n-1; next_vert = n-2;
        int cut_pos_backward = n-1; // the vertex at which we should cut the line (remove every vertex after this cut_pos)
        while (length_covered < 0.15 * total_length) {
            Vector3f curA = this->_vertices[line[cur_vert]]->position;
            Vector3f curB = this->_vertices[line[next_vert]]->position;
            Vector3f curC = this->_vertices[line[next_vert-1]]->position;
            Vector3f AC = curC - curA;
            Vector3f B_normal = this->_vertices[line[next_vert]]->normal;
            Vector3f AC_parallel =  AC.dot(B_normal) / (B_normal.norm()*B_normal.norm()) * B_normal; // AC projected onto the direction of normal

            Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
            if (acos(cur_tangent.dot(prev_tangent) / (cur_tangent.norm()*prev_tangent.norm())) > M_PI/4.0) {
                // remove everything before vertex B (starting from A)
                cut_pos_backward = next_vert;
            }
            // move on to the next segment
            prev_tangent = cur_tangent;
            length_covered += (curB - curA).norm();
            cur_vert--; next_vert--;
        }

        // remove everything before cut_pos_forward and after cut_pos_backward
        // need to modify: _vertices (vector<Vector3f>), _lines (vector<vector<int>>), _vertexNormals (vector<Vector3f>)
        vector<int> new_line;
        for (int i = cut_pos_forward; i <= cut_pos_backward; i++) {
            new_line.push_back(line[i]);
        }
        new_lines.push_back(new_line);

        // logical removal of vertices
        for (int i = 0; i < cut_pos_forward; i++) {
            _vertices[line[i]]->isActive = false;
        }

        for (int i = n-1; i > cut_pos_backward; i--) {
            _vertices[line[i]]->isActive = false;
        }
        itr++;
    }
    _lines = new_lines;

    // update the _vertices and _lines (using the correct indices)
    // - update _vertices
    unordered_map<int, int> index_map; // old index -> new index
    int active_number = 0;
    vector<Vertex*> new_vertices;
    for (int i = 0; i < _vertices.size(); i++) {
        if (_vertices[i]->isActive) {
            index_map[i] = active_number;
            active_number++;
            new_vertices.push_back(this->_vertices[i]);
        }else {
            delete this->_vertices[i];
        }
    }
    this->_vertices = new_vertices;

    // - remap the vertex indices in _lines
    vector<vector<int>> final_lines;
    for (auto &line : _lines) {
        vector<int> final_line;
        for (int vertex : line) {
            final_line.push_back(index_map[vertex]);
        }
        final_lines.push_back(final_line);
    }
    this->_lines = final_lines;
}

void Mesh::cleanUp() {
    for (int i = 0; i < this->_vertices.size(); i++) {
        delete this->_vertices[i];
    }
}

vector<vector<int>> Mesh::getLines() {
    return _lines;
}

// -------- PUBLIC ENDS -------------------------------------------------------------------------------




// -------- PRIVATE STARTS -------------------------------------------------------------------------------

vector<vector<int>> Mesh::parseToPolyline(vector<Vector2i> connections) {
    std::vector<std::vector<int>> polylines = std::vector<std::vector<int>>();
    int index = 0;
    while(index < connections.size()){
        vector<int> currentpoly = std::vector<int>();
        currentpoly.push_back(connections[index][0]);
        currentpoly.push_back(connections[index][1]);
        index = index + 1;
        while(connections[index][0] == currentpoly[currentpoly.size()-1]){
            currentpoly.push_back(connections[index][1]);
            index = index + 1;
        }
        polylines.push_back(currentpoly);
    }
    return polylines;
}

float Mesh::vertexVertexScore(Vertex* P, Vertex* Q, bool leftside) {
    Vector3f p = P->position;
    Vector3f q = Q->position;
    Vector3f tp = P->tangent;
    Vector3f tq = Q->tangent;
    Vector3f pb = (tp.cross(P->normal)).normalized();
    Vector3f qb = (tq.cross(Q->normal)).normalized();
    float da = (p-q).norm();
    float dt = 0.5*(abs((p-q).dot(tp)) + abs((p-q).dot(tq)));
    //This is specifically for a left match.
    Vector3f pc;
    if(leftside){
    pc = p - strokewidth*pb;
    }
    else{
        pc = p + strokewidth*pb;
    }

    Vector3f ql = q - strokewidth*qb;
    Vector3f qr = q + strokewidth*qb;

    Vector3f qc;
    if((ql-pc).norm() < (qr-pc).norm()){
        qc = ql;
    }
    else{
        qc = qr;
    }
    Vector3f mpqprime = 0.5*(pc+qc);
    Vector3f mpq = 0.5*(p+q);
    float dln = (mpq-mpqprime).norm();
    float finalscore = exp(-pow(da+dt+dln,2)/(2.f*pow(sigma, 2.f)));
    return finalscore;
}

// Pi_1 mean P_(i+1)
float Mesh::persistenceScore(Vertex* Pi, Vertex* Qi, Vertex* Pi_1, Vertex* Qi_1) {
    Vector3f pi_1 = Pi_1->position;
    Vector3f qi_1 = Qi_1->position;
    Vector3f pi = Pi->position;
    Vector3f qi = Qi->position;
    float dp = ((pi_1 - pi) - (qi_1 - qi)).norm() + ((pi_1 - qi) - (qi_1 - pi)).norm() + ((pi_1 - qi_1) - (pi - qi)).norm();
    float finalscore = exp(-pow(dp,2)/(2.f*pow(sigma, 2.f)));
    return finalscore;
}

// Intermediate M from step i-1 to i
// pi_1 means p_(i-1)
float Mesh::computeM(int pi, int qi, int pi_1, int qi_1, bool leftSide) {
    float vv = vertexVertexScore(_vertices[pi_1], _vertices[qi_1], leftSide);
    float pers = persistenceScore(_vertices[pi_1], _vertices[qi_1],  _vertices[pi], _vertices[qi]);
    return vv * pers;
}

/**
 * @brief perform one viterbi on one stroke S to find the sequence of q's that maximizes the objective function of M_l
 * @param vector<Vertex*> S: stroke to be processed
 * @param vector<vector<Vertex*>> candidates: a vector of candidates vector<C(pi)> for each vertex pi in S
 * @return vector<Vertex*> Q: sequence of q's, each qi is the optimal match for each pi in S
 */
vector<int> Mesh::viterbi(vector<int> S, vector<vector<int>> candidates, bool leftSide) {
    int k = 1; // time index
    int K = S.size(); // number of steps = number of vertices in S
    int M = 0; // max of number of candidates among all points pi
    for (int i = 0; i < candidates.size(); i++) {
        if (candidates[i].size() > M) {
            M = candidates[i].size();
        }
    }
    // construct a float[][] of size M*K
    float scores[M][K];
    // initialize the first column of dp to be all ones -- candidates for p0
    for (int i = 0; i < candidates[0].size(); i++) {
        scores[i][0] = 1.0;
    }
    // also need to store the current sequence of states at each q at time k -- an array of vectors, the array is of fixed size M
    vector<int> prev_sequences[M];
    vector<int> cur_sequences[M];
    // initialize the sequences to contain the starting points -- candidates for p0
    for (int i = 0; i < candidates[0].size(); i++) {
        scores[i][0] = 1.0;
    }
    // begin iterating through each step
    while (k < K) {
        for (int cur = 0; cur < candidates[k].size(); cur++) {
            // for each current q, select the prev that maximizes M_score so far
            // for prev, its M_score is stored in scores[prev][k-1]
            float cur_max = - 1e36;
            for (int prev = 0; prev < candidates[k-1].size(); prev++) {
                float stepM = computeM(S[k], candidates[k][cur], S[k-1], candidates[k-1][prev], leftSide);
                if (stepM * scores[prev][k-1] > cur_max) {
                    cur_max = stepM * scores[prev][k-1];
                    vector<int> newvec = prev_sequences[prev];
                    newvec.push_back(candidates[k][cur]);
                    cur_sequences[cur] = newvec; // extend from prev_seqence by appending cur
                }
            }
            // use cur_max as the score for cur
            scores[cur][k] = cur_max;
        }
        // updaet prev_sequences to be cur_sequences: prev_sequences = cur_sequences
        for (int i = 0; i < M; i++) {
            prev_sequences[i] = cur_sequences[i];
        }
        k++;
    }
    // now select the q that has the maximum score and retrieve its sequence
    // all final scores are in scores[][K-1]
    int final_index = 0;
    float max_score = 0;
    for (int i = 0; i < M; i++) {
        if (scores[i][K-1] > max_score) {
            max_score = scores[i][K-1];
            final_index = i;
        }
    }
    return cur_sequences[final_index];
}




// -------- PRIVATE ENDS -------------------------------------------------------------------------------

