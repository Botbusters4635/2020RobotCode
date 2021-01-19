//
// Created by abiel on 8/22/19.
//

#include "PCMManager.h"

PCMManager::PCMManager() : Manager("PCMManager") {
	log->info("Initializing PCMManager...");
	
	pistonsConfig = settings->getAllOfGroup("Pistons");
	
	initializePistons();
}

void PCMManager::initializePistons() {
	for (auto const &configEntry : pistonsConfig) {
		size_t propertySeparator = configEntry.first.find('.');
		
		std::string pistonName = configEntry.first.substr(0, propertySeparator);
		std::string propertyName = configEntry.first.substr(propertySeparator + 1);
		
		auto pistonIt = pistons.find(pistonName);
		
		if (pistonIt == pistons.end()) {
			pistonIt = pistons.emplace(pistonName, EctoPiston()).first;
		}
		
		if (propertyName == EctoPCMPropertyNames::id) {
			std::stringstream ss(configEntry.second);
			
			std::string token;
			std::vector<int> ids;
			while (std::getline(ss, token, ' ')) {
				ids.emplace_back(std::stoi(token));
			}
			
			if (ids.size() > 1) {
				pistonIt->second.isSingleSolenoid = false;
				pistonIt->second.aSolenoid = std::make_shared<frc::Solenoid>(ids.front());
				pistonIt->second.bSolenoid = std::make_shared<frc::Solenoid>(ids.at(1));
			} else if (!ids.empty()) {
				pistonIt->second.isSingleSolenoid = true;
				
				pistonIt->second.aSolenoid = std::make_shared<frc::Solenoid>(ids.front());
			} else {
				throw std::invalid_argument("No valid ID given for piston: " + pistonName);
			}
		}
		
		log->info("Added piston {}", pistonName);
	}
}

EctoPiston &PCMManager::getPiston(const std::string &pistonName) {
	auto piston = pistons.find(pistonName);
	
	if (piston == pistons.end()) {
		log->error("Invalid piston name: {}", pistonName);
		throw std::runtime_error(fmt::format("Invalid piston name: {}", pistonName));
	}
	
	return piston->second;
}

void PCMManager::setPistonState(EctoPiston &piston, bool newState) {
	piston.aSolenoid->Set(newState);
	piston.bSolenoid->Set(!newState);
	
	piston.currentState = newState;
}

void PCMManager::setPistonState(const std::string &pistonName, bool newState) {
	setPistonState(getPiston(pistonName), newState);
}

void PCMManager::togglePistonState(EctoPiston &piston) {
	setPistonState(piston, !piston.currentState);
}

void PCMManager::togglePistonState(const std::string &pistonName) {
	togglePistonState(getPiston(pistonName));
}

void PCMManager::update() {
	;
}