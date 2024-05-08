#include "settings.h"

Settings* Settings::settings = nullptr;

Settings* Settings::getInstance() {
    if (settings == nullptr) {
        settings = new Settings();
    }
    return settings;
}

void Settings::setIniFilePath(QString filePath) {
    this->iniFilePath = filePath.toStdString();
}

std::string Settings::getIniFilePath() {
   return this->iniFilePath;
}

int utils::elegantPair(int x, int y) {
    if (x != max(x,y)) {
        return pow(y,2) + x;
    }
    return pow(x,2) + x + y;
}

pair<int,int>  utils::elegantUnpair(int z) {
    int zSqrtFloored = (int)floor(sqrt(z));
    int zSqrtFlooredSquared = (int)pow(zSqrtFloored,2);
    if (z - zSqrtFlooredSquared < zSqrtFloored) {
        return make_pair(z - zSqrtFlooredSquared, zSqrtFloored);
    }
    return make_pair(zSqrtFloored, z - zSqrtFlooredSquared - zSqrtFloored);
}
