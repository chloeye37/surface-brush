#include "happly.h"
#include "ply_loader.h"
//#include "Eigen/StdVector"

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f); // not sure if this is necessary

vector<Vector3f> plyLoader::loadFromFile(string fileName) {
    happly::PLYData plyIn(fileName);
    vector<float> vertexNx = plyIn.getElement("vertex").getProperty<float>("nx");
    vector<float> vertexNy = plyIn.getElement("vertex").getProperty<float>("ny");
    vector<float> vertexNz = plyIn.getElement("vertex").getProperty<float>("nz");

    vector<Vector3f> vertexNormals;
    vertexNormals.reserve(vertexNx.size());
    for (int i = 0; i < vertexNx.size(); i++) {
        if (i % 3) continue;

        Vector3f normal(vertexNx[i], vertexNy[i], vertexNz[i]);
        vertexNormals.push_back(normal);
    }
    return vertexNormals;
}
