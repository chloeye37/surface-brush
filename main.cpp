#include <QCoreApplication>
#include <QCommandLineParser>
#include <QtCore>

#include <iostream>
#include <chrono>

#include "mesh.h"


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
//    QString outfile = settings.value("IO/outfile").toString();

    // Load
    Mesh m;
    m.loadFromFile(inObjFile.toStdString(), inPlyFile.toStdString());

    // Start timing
    auto t0 = std::chrono::high_resolution_clock::now();

    // TODO: our project code goes here

    // Finish timing
    auto t1 = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "Execution took " << duration << " milliseconds." << std::endl;

    // Save
//    m.saveToFile(outfile.toStdString());

    a.exit();
}
