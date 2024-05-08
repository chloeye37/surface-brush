#include "happly.h"
#include "ply_loader.h"

#include <iostream>
#include <fstream>
#include <array>
#include <set>
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

    vector<pair<Vector3f,float>> vertexNormalsAndStrokeWidths;
    vertexNormalsAndStrokeWidths.reserve(vertexNx.size());
    for (int i = 0; i < vertexNx.size(); i++) {
        if (i % 3) continue;

        float strokeWidth = 2.f * (Vector3f(vPos[i][0],vPos[i][1],vPos[i][2]) - Vector3f(vPos[neighborinds.at(i)][0],vPos[neighborinds.at(i)][1],vPos[neighborinds.at(i)][2])).norm();
        Vector3f normal(vertexNx[i], vertexNy[i], vertexNz[i]);
        vertexNormalsAndStrokeWidths.push_back(make_pair(normal, strokeWidth));
    }
    return vertexNormalsAndStrokeWidths;
}

//The below function loads a ply that was made with Blender.

vector<pair<Vector3f,float>> plyLoader::alternateloadFromFile(string fileName, vector<vector<int>> _lines, vector<Vector3f> vertices){
    happly::PLYData plyIn(fileName);

    std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
    std::vector<std::vector<size_t>> fInd = plyIn.getFaceIndices<size_t>();

    vector<float> vertexNx = plyIn.getElement("vertex").getProperty<float>("nx");
    vector<float> vertexNy = plyIn.getElement("vertex").getProperty<float>("ny");
    vector<float> vertexNz = plyIn.getElement("vertex").getProperty<float>("nz");

    //For each line, find the vertex that is really close to it in vpos

    float epsilon = 0.001;

    unordered_map<int,int> linetoply;

    //Fill linetoply
    for(int i = 0; i < _lines.size(); i++){
        vector<int> theline = _lines[i];
        for(int j = 0; j < theline.size(); j++){
            int theindex = theline[j];
            Vector3f theposition = vertices[theindex];
            for(int k = 0; k < vPos.size(); k++){
                float themag = sqrt(pow(static_cast<float>(vPos[k][0])-theposition[0],2)+
                                    pow(static_cast<float>(vPos[k][2])-theposition[1],2)+
                                    pow(static_cast<float>(-vPos[k][1])-theposition[2],2));
                if(themag < epsilon){
                    linetoply.emplace(theindex, k);
                }
            }
        }
    }

    vector<pair<Vector3f,float>> vertexNormalsAndStrokeWidths;
    for(int i = 0; i < _lines.size(); i++){
        vector<int> theline = _lines[i];
        //Each strip has a consistent stroke width
        //Calculate it first.
        float strokewidth;

        int ifirst = linetoply.at(theline[0]);
        int inext = linetoply.at(theline[1]);
        for(int j = 0; j < fInd.size(); j ++){
            //Go through each face. Find one that contains ifirst and inext
            vector<int> remainingindices;
            bool containsfirst = false;
            bool containsnext = false;
            for(int k = 0; k < 4; k++){
                //Make a set of all of the indices of fInd
                if(!((fInd[j][k] == ifirst) || (fInd[j][k] == inext))){
                    remainingindices.push_back(fInd[j][k]);
                }
                if(fInd[j][k] == ifirst){
                    containsfirst = true;
                }
                if(fInd[j][k] == inext){
                    containsnext = true;
                }
            }

            //Does it contain it? if so, get the first remaining index
            if(containsfirst && containsnext){
                Vector3f remainingpoint = Vector3f(vPos[remainingindices[0]][0],vPos[remainingindices[0]][1],vPos[remainingindices[0]][2]);
                //set the stroke width to be the minimum of the distances
                float tofirst = (remainingpoint-Vector3f(vPos[ifirst][0],vPos[ifirst][1],vPos[ifirst][2])).norm();
                float tonext = (remainingpoint-Vector3f(vPos[inext][0],vPos[inext][1],vPos[inext][2])).norm();
                if(tofirst < tonext){
                    strokewidth = 2*tofirst;
                }
                else{
                    strokewidth = 2*tonext;
                }
            }

        }
        //Now what? Find the face that contains those indices

        for(int j = 0; j < theline.size(); j++){
            int theindex = theline[j];
            //Get normals
            int plydex = linetoply.at(theindex);
            cout << to_string(strokewidth) << endl;
            vertexNormalsAndStrokeWidths.push_back(make_pair(Vector3f(vertexNx[plydex],vertexNz[plydex],-vertexNy[plydex]), strokewidth));
        }
    }

    return vertexNormalsAndStrokeWidths;
}
