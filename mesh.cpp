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

void Mesh::initFromVectors(const vector<Vector3f> &vertices,
                           const vector<vector<int>> &lines)
{
    // Copy vertices and faces into internal vector
    _vertices = vertices;
    _lines    = lines;
}

void Mesh::loadFromFile(const std::string &inObjFilePath, const std::string &inPlyFilePath)
{
    // load obj file
    pair<vector<Vector3f>, vector<Vector2i>> verticesAndLineSegments = objLoader::loadFromFile(inObjFilePath);
    vector<Vector3f> vertices = verticesAndLineSegments.first;
    vector<Vector2i> lineSegments = verticesAndLineSegments.second;
    vector<vector<int>> lines = this->parseToPolyline(lineSegments);

    // load ply file
    vector<Vector3f> normals = plyLoader::loadFromFile(inPlyFilePath);

    // populate the _m_vertices vector
    vector<Vertex> m_vertices;
    for (int i = 0; i < vertices.size(); i++) {
        Vertex vertex;
        vertex.isActive = true;
        vertex.position = vertices[i];
        vertex.normal = normals[i];
        // vertex.tangent is set later in calculateTangents()
        m_vertices.push_back(vertex);
    }

    this->_vertices = vertices;
    this->_lines = lines;
    this->_vertexNormals = normals;
    this->_m_vertices = m_vertices;

    calculateTangents();
}

void Mesh::calculateTangents() {
    // loop through all lines
    for (auto & line : _lines) {
        int n = line.size();
        // loop through all vertices on the line
        Vector3f first_tangent = _vertices[line[1]] - _vertices[line[0]]; // tangent of the first vertex is just the line segment direction
        _m_vertices[line[0]].tangent = first_tangent;

        for (int i = 1; i < n - 1; i++) {
            Vector3f curA = _vertices[line[i-1]];
            Vector3f curB = _vertices[line[i]];
            Vector3f curC = _vertices[line[i+1]];
            Vector3f AC = curC - curA;
            Vector3f B_normal = _vertexNormals[line[i]];
            Vector3f AC_parallel =  AC.dot(B_normal) / (B_normal.norm()*B_normal.norm()) * B_normal; // AC projected onto the direction of normal

            Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
            _m_vertices[line[i]].tangent = cur_tangent;

        }
        Vector3f last_tangent = _vertices[line[n-1]] - _vertices[line[n-2]]; // tangent of the last vertex is just the line segment direction
        _m_vertices[line[n-1]].tangent = last_tangent;

    }

}

void Mesh::saveToFile(const string &outStrokeFilePath, const string &outMeshFilePath)
{
    ofstream outStrokeFile;
    outStrokeFile.open(outStrokeFilePath);

    ofstream outMeshFile;
    outMeshFile.open(outMeshFilePath);

    // Write vertices
    for (size_t i = 0; i < _m_vertices.size(); i++)
    {
        Vertex v = _m_vertices[i];
        outMeshFile << "v " << v.position[0] << " " << v.position[1] << " " << v.position[2] << endl;
        outStrokeFile << "v " << v.position[0] << " " << v.position[1] << " " << v.position[2] << endl;

    }

    // Write vertex normals
    for (size_t i = 0; i < _m_vertices.size(); i++)
    {
        Vertex v = _m_vertices[i];
        outMeshFile << "vn " << v.normal[0] << " " << v.normal[1] << " " << v.normal[2] << endl;

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
    // Have: vector<vector<int>> _lines;  vector<Vector3f> _vertexNormals;
    vector<vector<int>> new_lines;
    int itr = 0;
//    std::cout << "Number of lines: " << _lines.size() << std::endl;
    // process each line at a time
    for (auto & line : _lines) {
        // first calculate line length -- sum of all segment lengths
        float total_length = 0;
        for (int i = 1; i < line.size(); i++) {
            // distance between vertex i and vertex i-1
            total_length += (_vertices[line[i]] - _vertices[line[i-1]]).norm();
        }

        float length_covered = 0;
        // forward direction: start to end until length_covered > 0.15 * total_length
        // calculate the tangents along the way and test if they differ by more than 45 degrees
        Vector3f prev_tangent = _vertices[line[1]] - _vertices[line[0]]; // tangent of the first vertex is just the line segment direction
        int cur_vert = 0; int next_vert = 1;
        int cut_pos_forward = 0; // the vertex at which we should cut the line (remove every vertex before this cut_pos)
        while (length_covered < 0.15 * total_length) {
            Vector3f curA = _vertices[line[cur_vert]];
            Vector3f curB = _vertices[line[next_vert]];
            Vector3f curC = _vertices[line[next_vert+1]];
            Vector3f AC = curC - curA;
            Vector3f B_normal = _vertexNormals[line[next_vert]];
            Vector3f AC_parallel =  AC.dot(B_normal) / (B_normal.norm()*B_normal.norm()) * B_normal; // AC projected onto the direction of normal

            Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
            if (acos(cur_tangent.dot(prev_tangent) / (cur_tangent.norm()*prev_tangent.norm())) > M_PI/4.0) {
                // remove everything before vertex B (starting from A)
                cut_pos_forward = next_vert;
//                std::cout << "--------inside removing condition" << std::endl;
            }
            // move on to the next segment
            prev_tangent = cur_tangent;
            length_covered += (curB - curA).norm();
            cur_vert++; next_vert++;
        }

        // backward direction: end to start
        int n = line.size();
        length_covered = 0;
        prev_tangent = _vertices[line[n-2]] - _vertices[line[n-1]]; // tangent of the first vertex is just the line segment direction
        cur_vert = n-1; next_vert = n-2;
        int cut_pos_backward = n-1; // the vertex at which we should cut the line (remove every vertex after this cut_pos)
        while (length_covered < 0.15 * total_length) {
            Vector3f curA = _vertices[line[cur_vert]];
            Vector3f curB = _vertices[line[next_vert]];
            Vector3f curC = _vertices[line[next_vert-1]];
            Vector3f AC = curC - curA;
            Vector3f B_normal = _vertexNormals[line[next_vert]];
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

        // remove vertices and normals from _vertices and _vertexNormals
        for (int i = 0; i < cut_pos_forward; i++) {
            _m_vertices[line[i]].isActive = false;
//            if (line[i] <= 3000) std::cout << line[i] << std::endl;
        }

        for (int i = n-1; i > cut_pos_backward; i--) {
            _m_vertices[line[i]].isActive = false;
//            if (line[i] <= 3000) std::cout << line[i] << std::endl;
        }
        itr++;
    }
    _lines = new_lines;
    // update the _m_vertices and _lines (using the correct indices)
    // first delete from _lines according to isActive -- done in the previous loop
    // then erase from _m_vertices and process _lines again
    // create reindexing map
    std::map<int, int> index_map; // old index -> new index
    vector<int> remove_indices;
    int active_number = 0;
    for (int i = 0; i < _m_vertices.size(); i++) {
        if (_m_vertices[i].isActive) {
            index_map[i] = active_number;
            active_number++;
        }else {
            remove_indices.push_back(i);
        }
    }
    // delete from _m_vertices
    for (int index : remove_indices) {
        _m_vertices.erase(_m_vertices.begin() + index - 1);
    }
    // remap the vertex indices in _lines
    vector<vector<int>> final_lines;
    for (auto &line : _lines) {
        vector<int> final_line;
        for (int vertex : line) {
            final_line.push_back(index_map[vertex]);
        }
        final_lines.push_back(final_line);
    }
    _lines = final_lines;
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
    //One note. I'm not sure if this will add the last polyline. If not, run polylines.push_back one more time right here:
    return polylines;
}

float Mesh::vertexVertexScore(Vertex P, Vertex Q, bool leftside){
    Vector3f p = P.position;
    Vector3f q = Q.position;
    Vector3f tp = P.tangent;
    Vector3f tq = Q.tangent;
    Vector3f pb = (tp.cross(P.normal)).normalize();
    Vector3f qb = (tq.cross(Q.normal)).normalize();
    float da = (p-q).norm();
    float dt = 0.5*(((p-q).dot(tp)).norm()+((p-q).dot(tq)).norm());
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
    float finalscore = exp(-pow(da+dt+dln,2)/(2.f*sigma^2));
    return finalscore;
}

// -------- PRIVATE ENDS -------------------------------------------------------------------------------

