#pragma once

#include <vector>

#include "Eigen/StdVector"
#include "Eigen/Dense"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3i);

class Mesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void initFromVectors(const std::vector<Eigen::Vector3f> &vertices,
                         const std::vector<Eigen::Vector2i> &faces);

    void loadFromFile(const std::string &filePath);
    void saveToFile(const std::string &filePath);

private:
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector2i> _faces;
};
