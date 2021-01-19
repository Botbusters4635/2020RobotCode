//
// Created by abiel on 8/22/19.
//

#ifndef BOTBUSTERSREBIRTH_ECTOPCM_H
#define BOTBUSTERSREBIRTH_ECTOPCM_H

#include <frc/Solenoid.h>
#include <Core/EctoModule/Manager.h>

#include <string>

struct EctoPiston {
	bool isSingleSolenoid = false;
	
	std::shared_ptr<frc::Solenoid> aSolenoid;
	std::shared_ptr<frc::Solenoid> bSolenoid;
	
	bool currentState = false;
};

class PCMManager : public Manager<PCMManager> {
	friend class Manager<PCMManager>;

public:
	EctoPiston &getPiston(const std::string &pistonName);
	
	static void setPistonState(EctoPiston &piston, bool newState);
	
	void setPistonState(const std::string &pistonName, bool newState);
	
	static void togglePistonState(EctoPiston &piston);
	
	void togglePistonState(const std::string &pistonName);

protected:
	void update() override;

private:
	PCMManager();
	
	PCMManager &operator=(const PCMManager &);
	
	void initializePistons();
	
	std::map<std::string, std::string> pistonsConfig;
	std::map<std::string, EctoPiston> pistons;
};

namespace EctoPCMPropertyNames {
	const std::string name = "name";
	const std::string id = "id";
}

#endif //BOTBUSTERSREBIRTH_ECTOPCM_H
