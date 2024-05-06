#pragma once
#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <QtCore>
using namespace std;

// singleton
// not for multithread program
// reference: https://refactoring.guru/design-patterns/singleton/cpp/example
class Settings {
protected:
    Settings(){};
    std::string iniFilePath;
public:
    Settings(Settings &other) = delete;
    void operator=(const Settings &) = delete;

    static Settings* settings;
    static Settings* getInstance();

    void setIniFilePath(QString filePath);
    std::string getIniFilePath();

    // easily accessible fields
    std::string inObjFile;
    std::string inPlyFile;
    std::string outStrokeFile;
    std::string outMeshFile;
    bool isDebug = false;
    int noOfNearEndVerticesToConsider = 3;
};

namespace utils {
    // using Anh's code from Mesh
    // Elegant pairing & unpairing of 2 NON-NEGATIVE INTEGERS
    // reference: http://szudzik.com/ElegantPairing.pdf
    // REMINDER: when using this in this project, always pass in x < y
    int elegantPair(int x, int y);

    pair<int,int> elegantUnpair(int z);
}

#endif // UTILS_H
