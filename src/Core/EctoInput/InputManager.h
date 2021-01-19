/*
* Settings
* Group: Input
* StickPort: Stick Port (probably 0)
* Deadzone: Deadzone for Joystick
* ButtonName# - ButtonName#: Button Name
* AxisName# - AxisName#: Axis Name
*/

#ifndef BOTBUSTERSREBIRTH_ECTOINPUT_H
#define BOTBUSTERSREBIRTH_ECTOINPUT_H

#include <Core/EctoInput/Buttons/EctoButton.h>
#include <Core/EctoInput/Axis/JoystickAxis.h>
#include <Core/EctoModule/Manager.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <map>
#include <utility>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>

/**
 * This class is in charge of managing and updating all EctoButton and JoystickAxis classes for use within other systems.
 */
class InputManager : public Manager<InputManager> {
	friend class Manager<InputManager>;

public:
	void registerButton(EctoButton *button, const std::string &buttonName, int joystickId = 1);
	
	void registerAxis(JoystickAxis *axis, const std::string &axisName, int joystickId = 1);
	
	/**
	 * Rumble values should be from 0 to 1
	 */
	void setControllerRumble(double leftRumble, double rightRumble, int joystickId = 1);
	
	int getPOVAngleReading(int joystickId = 1) const;

protected:
	void update() override;

private:
	struct JoystickData {
		std::shared_ptr<frc::Joystick> joystick;
		
		int joystickId;
		int wpiJoystickId;
		
		int axisCount;
		int buttonCount;
		
		//Stores names to ids
		std::map<std::string, int> buttonNames;
		std::map<std::string, int> axesNames;
		
		std::map<int, std::vector<std::shared_ptr<EctoButton>>> buttonsToUpdate;
		std::map<int, std::vector<std::shared_ptr<JoystickAxis>>> axesToUpdate;
	};
	
	bool isButtonPressed(int buttonID, int joystickID = 1) const;
	
	double readAxisValue(int axisID, int joystickID = 1) const;
	
	InputManager();
	
	InputManager &operator=(const InputManager &);
	
	void initializeJoysticks();
	
	void initializeButtons();
	
	void initializeAxis();
	
	const std::string baseNameStick = "StickPort";
	const std::string baseNameStickSelector = "Stick";
	const std::string baseNameAxis = "AxisName";
	const std::string baseNameButton = "ButtonName";
	
	int joystickCount;
	
	std::map<int, JoystickData> joysticksData;
	
	mutable std::mutex ectoInputLock;
	frc::DriverStation &driverStation = frc::DriverStation::GetInstance();
};

#endif
