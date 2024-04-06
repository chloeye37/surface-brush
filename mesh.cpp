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

    // initialize the isActive vector to be all true
    vector<bool> isActive;
    for (int i = 0; i < vertices.size(); i++) {
        isActive.push_back(true);
    }

    this->_vertices = vertices;
    this->_lines = lines;
    this->_vertexNormals = normals;
    this->_isActive = isActive;
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
        const Vector3f &v = _vertices[i];
        outMeshFile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
    }

    // Write vertex normals
    for (size_t i = 0; i < _vertexNormals.size(); i++)
    {
        const Vector3f &n = _vertexNormals[i];
        outMeshFile << "vn " << n[0] << " " << n[1] << " " << n[2] << endl;
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
            _isActive[line[i]] = false;
//            _vertices.erase(_vertices.begin() + line[i] - 1); //erase the (line[i]+1-1)th element
//            _vertexNormals.erase(_vertexNormals.begin() + line[i] - 1);
        }

        for (int i = n-1; i > cut_pos_backward; i--) {
            _isActive[line[i]] = false;
//            _vertices.erase(_vertices.begin() + line[i] - 1); // erase the (line[i]+1-1)th element
//            _vertexNormals.erase(_vertexNormals.begin() + line[i] - 1);
        }
//        std::cout << "After: vertices number: " << _vertices.size() << std::endl;
        itr++;
    }
    _lines = new_lines;
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

// -------- PRIVATE ENDS -------------------------------------------------------------------------------

