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
