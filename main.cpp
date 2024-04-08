#include <QCoreApplication>
#include <QCommandLineParser>
#include <QtCore>

#include <iostream>
#include <chrono>

#include "mesh.h"


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
        index = index + 1;
    }
    //One note. I'm not sure if this will add the last polyline. If not, run polylines.push_back one more time right here:
    return polylines;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("config",  "Path of the config (.ini) file.");
    parser.process(a);

    // Check for invalid argument count
    const QStringList args = parser.positionalArguments();
    if (args.size() < 1) {
        std::cerr << "Not enough arguments. Please provide a path to a config file (.ini) as a command-line argument." << std::endl;
        a.exit(1);
        return 1;
    }

    // Parse common inputs
    QSettings settings( args[0], QSettings::IniFormat );
    QString inObjFile  = settings.value("IO/inObjFile").toString();
    QString inPlyFile  = settings.value("IO/inPlyFile").toString();
    QString outStrokeFile = settings.value("IO/outStrokeFile").toString();
    QString outMeshFile = settings.value("IO/outMeshFile").toString();


    // Load
    Mesh m;
    m.loadFromFile(inObjFile.toStdString(), inPlyFile.toStdString());

    // // Start timing
    // auto t0 = std::chrono::high_resolution_clock::now();

    // TODO: our project code goes here
    m.preprocessLines();
    std::cout << "Strips preprocessed." << std::endl;

    // // Finish timing
    // auto t1 = std::chrono::high_resolution_clock::now();
    // auto duration = duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    // std::cout << "Execution took " << duration << " milliseconds." << std::endl;

    // Save
    m.saveToFile(outStrokeFile.toStdString(), outMeshFile.toStdString());
    std::cout << "Saved to file." << std::endl;

    // Clean up
    m.cleanUp();

    // a.exit();
}
