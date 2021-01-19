//
// Created by hiram on 3/07/19.
//
#include "Module.h"

std::string Module::configRootDir;


Module::Module(const std::string& name, bool useConfigFile) : name(name){
    if(useConfigFile){
        settings = std::make_unique<EctoSettings>(configRootDir + name + ".ini");
    }

    log = spdlog::stdout_color_mt(name);
    log->info("Starting...");
}

std::string Module::getName() {
    return name;
}

std::string Module::getConfigFileRootDir() {
    return Module::configRootDir;
}

void Module::setConfigFileRootDir(const std::string &configRootDir){
    Module::configRootDir = configRootDir;
}