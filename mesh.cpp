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

    this->_vertices = vertices;
    this->_lines = lines;
    this->_vertexNormals = normals;
}

void Mesh::saveToFile(const string &filePath)
{
//    ofstream outfile;
//    outfile.open(filePath);

//    // Write vertices
//    for (size_t i = 0; i < _vertices.size(); i++)
//    {
//        const Vector3f &v = _vertices[i];
//        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
//    }

//    // Write faces
//    for (size_t i = 0; i < _faces.size(); i++)
//    {
//        const Vector3i &f = _faces[i];
//        outfile << "f " << (f[0]+1) << " " << (f[1]+1) << " " << (f[2]+1) << endl;
//    }

//    outfile.close();
}

// -------- PUBLIC ENDS -------------------------------------------------------------------------------




// -------- PRIVATE STARTS -------------------------------------------------------------------------------

vector<vector<int>> Mesh::parseToPolyline(vector<Vector2i> connections) {
    std::vector<std::vector<int>> polylines = std::vector<std::vector<int>>();
    int index = 0;
    while(index < connections.size()){
        std::vector<int> currentpoly = std::vector<int>();
        currentpoly.push_back(connections[index][0]);
        currentpoly.push_back(connections[index][1]);
        while(connections[index+1][0] == currentpoly[currentpoly.size()-1]){
            currentpoly.push_back(connections[index+1][1]);
            index = index + 1;
        }
        polylines.push_back(currentpoly);
    }
    //One note. I'm not sure if this will add the last polyline. If not, run polylines.push_back one more time right here:
    return polylines;
}

void Mesh::loadIntoDataStructure() {
    //
}

// -------- PRIVATE ENDS -------------------------------------------------------------------------------

