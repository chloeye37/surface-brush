#include <QCoreApplication>
#include <QCommandLineParser>
#include <QtCore>

#include <iostream>
#include <chrono>

#include "mesh.h"
#include "util/settings.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("config", "Path of the config (.ini) file.");
    parser.process(a);

    // Check for invalid argument count
    const QStringList args = parser.positionalArguments();
    if (args.size() < 1)
    {
        std::cerr << "Not enough arguments. Please provide a path to a config file (.ini) as a command-line argument." << std::endl;
        a.exit(1);
        return 1;
    }

    // Parse common inputs
    QSettings settings(args[0], QSettings::IniFormat);
    Settings *appSettings = Settings::getInstance();
    appSettings->setIniFilePath(args[0]);
    appSettings->inObjFile = settings.value("IO/inObjFile").toString().toStdString();
    appSettings->inPlyFile = settings.value("IO/inPlyFile").toString().toStdString();
    appSettings->outStrokeFile = settings.value("IO/outStrokeFile").toString().toStdString();
    appSettings->outMeshFile = settings.value("IO/outMeshFile").toString().toStdString();
    appSettings->isDebug = settings.value("Debug/isDebug").toBool();
    appSettings->noOfNearEndVerticesToConsider = settings.value("Params/noOfNearEndVerticesToConsider").toInt();

    // Load
    Mesh m;
    m.loadFromFile();

    // Start timing
    auto t0 = std::chrono::high_resolution_clock::now();

    // Main algo
    m.preprocessLines();
    std::cout << "Strips preprocessed." << std::endl;
    m.getRestrictedMatchingCandidates();
    std::cout << "Got restricted matching candidates." << std::endl;
    m.getMatches();
    std::cout << "Finished matching." << std::endl;
    m.meshStripGeneration();
    std::cout << "Generated mesh strips." << std::endl;
    m.computeBoundaries();
    std::cout << "Computed boundaries." << std::endl;

    int smoothiterations = 0;
    for(int i = 0; i < smoothiterations; i++){
        m.smoothBoundaries();
    }
    std::cout << "Smoothed boundaries." << std::endl;

    // ------ Debug undecided triangle generation --------
//    m.computeUndecidedTriangles();
//    std::cout << "Computed undecided triangles." << std::endl;
    m.manifoldConsolidation();
    std::cout << "Fixed mesh to be a manifold mesh." << std::endl;


    // Finish timing
    auto t1 = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "Execution took " << duration << " milliseconds." << std::endl;

    // Save
    //    m.saveToFile(outStrokeFile.toStdString(), outMeshFile.toStdString());

    //NOTE!!!! Debug save to file currently does not really save faces in a way that makes sense, mostly because matches
    //between edges are directed(Chloe knows about this). One way to make sure we don't double coumt faces is to make a hash
    //out of each face's indices
    m.debugSaveToFile();
//    m.debugUndecidedTrianglesSaveToFile();
//    m.debugIncompatibleTrianglesSaveToFile();
    std::cout << "Saved to file." << std::endl;

    // Clean up
    m.cleanUp();

    a.exit();
}
