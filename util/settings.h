#pragma once
#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <QtCore>

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
};

#endif // UTILS_H
