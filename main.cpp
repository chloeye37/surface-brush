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
    std::cout << "Hey there" << std::endl;
    std::vector<Eigen::Vector2i> connectiontest = std::vector<Eigen::Vector2i>();
    connectiontest.push_back(Eigen::Vector2i(0,1));
    connectiontest.push_back(Eigen::Vector2i(1,2));
    connectiontest.push_back(Eigen::Vector2i(2,3));
    connectiontest.push_back(Eigen::Vector2i(4,5));
    connectiontest.push_back(Eigen::Vector2i(5,6));
    connectiontest.push_back(Eigen::Vector2i(7,8));
    connectiontest.push_back(Eigen::Vector2i(9,10));

    std::cout << "about to parse" << std::endl;

    std::vector<std::vector<int>> polyout = parse_to_polyline(connectiontest);

    std::cout << "number of polylines: " + std::to_string(polyout.size()) << std::endl;

    std::string outstring = "[";
    for(int i = 0; i < polyout.size(); i++){
        outstring = outstring + "(";
        for(int j = 0; j < polyout[i].size(); j++){
            outstring = outstring + std::to_string(polyout[i][j]) + ", ";
        }
        outstring = outstring + "), ";
    }

    outstring = outstring + "]";
    std::cout << outstring << std::endl;

    // QCoreApplication a(argc, argv);
    // QCommandLineParser parser;
    // parser.addHelpOption();
    // parser.addPositionalArgument("config",  "Path of the config (.ini) file.");
    // parser.process(a);

    // // Check for invalid argument count
    // const QStringList args = parser.positionalArguments();
    // if (args.size() < 1) {
    //     std::cerr << "Not enough arguments. Please provide a path to a config file (.ini) as a command-line argument." << std::endl;
    //     a.exit(1);
    //     return 1;
    // }

    // // Parse common inputs
    // QSettings settings( args[0], QSettings::IniFormat );
    // QString infile  = settings.value("IO/infile").toString();
    // QString outfile = settings.value("IO/outfile").toString();
    // QString method  = settings.value("Method/method").toString();

    // // A note about the representations of other parameters in the .ini files for the various methods:

    // // args1:
    // // Subdivide: number of iterations
    // // Simplify:  number of faces to remove
    // // Remesh:    number of iterations
    // // Denoise:   number of iterations

    // // args2:
    // // Remesh: Tangential smoothing weight
    // // Denoise: Smoothing parameter 1 (\Sigma_c)

    // // args3:
    // // Denoise: Smoothing parameter 2 (\Sigma_s)

    // // args4:
    // // Denoise: Kernel size (\rho)


    // // Load
    // Mesh m;
    // m.loadFromFile(infile.toStdString());

    // // Start timing
    // auto t0 = std::chrono::high_resolution_clock::now();

    // // Switch on method
    // if (method == "subdivide") {
    //     int numIterations = settings.value("Parameters/args1").toInt();

    //     // TODO

    // } else if (method == "simplify") {

    //     // TODO

    // } else if (method == "remesh") {

    //     // TODO

    // } else if (method == "noise") {

    //     // TODO

    // } else if (method == "denoise") {

    //     // TODO

    // } else {

    //     std::cerr << "Error: Unknown method \"" << method.toUtf8().constData() << "\"" << std::endl;

    // }

    // // Finish timing
    // auto t1 = std::chrono::high_resolution_clock::now();
    // auto duration = duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    // std::cout << "Execution took " << duration << " milliseconds." << std::endl;

    // // Save
    // m.saveToFile(outfile.toStdString());

    // a.exit();
}
