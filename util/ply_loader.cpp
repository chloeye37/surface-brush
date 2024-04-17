#include "happly.h"
#include "ply_loader.h"
//#include "Eigen/StdVector"

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f); // not sure if this is necessary

vector<pair<Vector3f,float>> plyLoader::loadFromFile(string fileName) {
    happly::PLYData plyIn(fileName);

    std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
    std::vector<std::vector<size_t>> fInd = plyIn.getFaceIndices<size_t>();

    vector<float> vertexNx = plyIn.getElement("vertex").getProperty<float>("nx");
    vector<float> vertexNy = plyIn.getElement("vertex").getProperty<float>("ny");
    vector<float> vertexNz = plyIn.getElement("vertex").getProperty<float>("nz");

    std::unordered_map<int,int> neighborinds;

    for(int i = 0; i < fInd.size(); i++){
        if(i % 2 == 0){
            if(!neighborinds.contains(fInd[i][0])){
                neighborinds.emplace(fInd[i][0],fInd[i][1]);
            }
            if(!neighborinds.contains(fInd[i][3])){
                neighborinds.emplace(fInd[i][3],fInd[i][2]);
            }
        }
    }

    vector<pair<Vector3f,float>> vertexNormals;
    vertexNormals.reserve(vertexNx.size());
    for (int i = 0; i < vertexNx.size(); i++) {
        if (i % 3) continue;

        Vector3f normal(vertexNx[i], vertexNy[i], vertexNz[i]);
        // std::cout << "about to search for " + std::to_string(neighborinds.at(i)) << std::endl;
        float dist = (Vector3f(vPos[i][0],vPos[i][1],vPos[i][2]) - Vector3f(vPos[neighborinds.at(i)][0],vPos[neighborinds.at(i)][1],vPos[neighborinds.at(i)][2])).norm();
        // std::cout << "strokewidth" + std::to_string(dist) << std::endl;
        vertexNormals.push_back(std::make_pair(normal, dist));
    }
    return vertexNormals;
}
