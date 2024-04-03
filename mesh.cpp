#include "mesh.h"

#include <iostream>
#include <fstream>

#include <QFileInfo>
#include <QString>
#include <Eigen>

#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"
#include "util/ply_loader.h"

#include <fstream>

using namespace Eigen;
using namespace std;

void Mesh::initFromVectors(const vector<Vector3f> &vertices,
                           const vector<Vector3i> &faces)
{
    // Copy vertices and faces into internal vector
    _vertices = vertices;
    _faces    = faces;
}

void Mesh::loadFromFile(const std::string &inObjFilePath, const std::string &inPlyFilePath)
{
//    tinyobj::attrib_t attrib;
//    vector<tinyobj::shape_t> shapes;
//    vector<tinyobj::material_t> materials;

//    QFileInfo info(QString(filePath.c_str()));
//    string err;
//    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
//                                info.absoluteFilePath().toStdString().c_str(), (info.absolutePath().toStdString() + "/").c_str(), true);
//    if (!err.empty()) {
//        cerr << err << endl;
//    }

//    if (!ret) {
//        cerr << "Failed to load/parse .obj file" << endl;
//        return;
//    }

//    for (size_t s = 0; s < shapes.size(); s++) {
//        size_t index_offset = 0;
//        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
//            unsigned int fv = shapes[s].mesh.num_face_vertices[f];

//            Vector3i face;
//            for (size_t v = 0; v < fv; v++) {
//                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

//                face[v] = idx.vertex_index;
//            }
//            _faces.push_back(face);

//            index_offset += fv;
//        }
//    }
//    for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
//        _vertices.emplace_back(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]);
//    }
//    cout << "Loaded " << _faces.size() << " faces and " << _vertices.size() << " vertices" << endl;

    // load ply file
    vector<Vector3f> normals = plyLoader::loadFromFile(inPlyFilePath);
}

void Mesh::saveToFile(const string &filePath)
{
    ofstream outfile;
    outfile.open(filePath);

    // Write vertices
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        const Vector3f &v = _vertices[i];
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
    }

    // Write faces
    for (size_t i = 0; i < _faces.size(); i++)
    {
        const Vector3i &f = _faces[i];
        outfile << "f " << (f[0]+1) << " " << (f[1]+1) << " " << (f[2]+1) << endl;
    }

    outfile.close();
}

//
std::vector<std::vector<int>> parse_to_polyline(std::vector<Eigen::Vector2i> connections){
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
