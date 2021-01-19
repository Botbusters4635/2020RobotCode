//
// Created by abiel on 1/5/20.
//

#ifndef BOTBUSTERSREBIRTH_DROIDEKA_H
#define BOTBUSTERSREBIRTH_DROIDEKA_H

#include "Core/EctoRobot.h"
#include "Systems/TimingDataPublisher/TimingDataPublisher.h"
#include "Commands/WpiTrajectoryFollower/WpiTrajectoryFollower.h"
#include "Systems/ControlPanelHandler/ControlPanelHandler.h"
#include "Utilities/MotorCharacterization/MotorCharacterizationHelper.h"

#include "Systems/PIDShooter/PidShooter.h"

#include <Core/EctoInput/Buttons/EctoButton.h>
#include <Core/EctoInput/InputManager.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Systems/EctoSwerve/EctoSwerveInputHandler.h"

#include "Systems/BallInputHandler/BallInputHandler.h"
#include "Systems/BallFeeder/BallFeeder.h"
#include "Systems/BallIntake/BallIntake.h"
#include "Systems/Elevator/Elevator.h"
#include "Systems/ElevatorInputHandler/ElevatorInputHandler.h"

#include <Core/PCM/PCMManager.h>
#include <Core/EctoInput/InputManager.h>

#include "Commands/FeedBalls/FeedBalls.h"
#include "Commands/SpinupFlywheel/SpinupFlywheel.h"
#include "Commands/LowerIntake/LowerIntake.h"
#include "Systems/ShooterCharacterizer/ShooterCharacterizer.h"

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>

#include <frc/DutyCycle.h>
#include <frc/AnalogInput.h>

#include "Core/LEDManager/LedManager.h"
#include "Core/VisionManager/VisionManager.h"

#include "Autos/BasicAuto.h"
#include "Systems/TeleopPathHelper/TeleopPathHelper.h"

#include "Systems/SwerveErrorTester/SwerveErrorTester.h"

class Droideka : public EctoRobot {
public:
	Droideka();
	
	void autoInit() override;
	
	void autoUpdate() override;
	
	void teleopInit() override;
	
	void teleopUpdate() override;

private:
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	PCMManager &pcm = PCMManager::getInstance();
	MotorManager &handler = MotorManager::getInstance();
	
	LEDManager &ledManager = LEDManager::getInstance();
	VisionManager &visionManager = VisionManager::getInstance();
	
	InputManager &input = InputManager::getInstance();
	
	std::shared_ptr<EctoSwerve> swerve;
	std::shared_ptr<EctoSwerveInputHandler> swerveInputHandler;
	
	std::shared_ptr<PIDShooter> shooter;
	
	std::shared_ptr<BallFeeder> ballFeeder;
	std::shared_ptr<BallInputHandler> ballInputHandler;
	
	std::shared_ptr<BallIntake> ballIntake;
	std::shared_ptr<Elevator> elevator;
	std::shared_ptr<ElevatorInputHandler> elevatorInputHandler;
	
	std::shared_ptr<EctoMotor> leftFlywheel, rightFlywheel;
	
	std::shared_ptr<EctoMotor> leftIntake, rightIntake;
	
	std::shared_ptr<WPITrajectoryFollower> testPath;
	//std::unique_ptr<FeedBalls> testBallFeeder;
	//std::unique_ptr<SpinupFlywheel> spinupFlywheelCommand;
	
	std::shared_ptr<frc2::SequentialCommandGroup> shootBalls;
	frc::Trajectory trajectory, trajectory2, trajectory3;
	WPITrajectoryFollowerConfig trajConfig;
	
	
	std::shared_ptr<ShooterCharacterizer> characterizer;
	
	frc::AnalogInput rightInput{1};
	frc::AnalogInput leftInput{0};
	
	std::shared_ptr<BasicAuto> basicTestAuto;
	
	std::shared_ptr<TeleopPathHelper> pathHelper;
	
	std::shared_ptr<SwerveErrorTester> errorTester;
};


#endif //BOTBUSTERSREBIRTH_DROIDEKA_H
