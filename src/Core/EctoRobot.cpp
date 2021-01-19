//
// Created by hiram on 28/06/19.
//

#include "EctoRobot.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/livewindow/LiveWindow.h>
#include <memory>

EctoRobot::EctoRobot(const std::string &robotName) : TimedRobot(units::second_t(0.005)), System(robotName) {
	systemManager = std::make_shared<SystemHandler>("SystemManager");
	inputHandlers = std::make_shared<SystemHandler>("InputHandlers");
	spdlog::set_level(spdlog::level::trace);
	
	systemManager->addSubsystem(managerHandler);
	
	systemManagerTimingPublisher = std::make_shared<TimingDataPublisher>(systemManager, "SystemTimingData");
	inputHandlerTimingPublisher = std::make_shared<TimingDataPublisher>(inputHandlers, "InputHandlerTimingData");
	
	systemManager->addSubsystem(systemManagerTimingPublisher);
	systemManager->addSubsystem(inputHandlerTimingPublisher);
}

void EctoRobot::RobotInit() {
	log->info("RobotInit...");
	frc::LiveWindow::GetInstance()->DisableAllTelemetry();
	
	systemManager->initRobot();
	inputHandlers->initRobot();
}


void EctoRobot::RobotPeriodic() {
	systemManager->updateRobot();
}

//TODO Run command handler here
void EctoRobot::AutonomousInit() {
	frc2::CommandScheduler::GetInstance().Enable();
	log->info("AutonomousInit... ");
	autoInit();
}

void EctoRobot::AutonomousPeriodic() {
	frc2::CommandScheduler::GetInstance().Run();
	
	autoUpdate();
}

void EctoRobot::TeleopInit() {
	frc2::CommandScheduler::GetInstance().Disable();
	frc2::CommandScheduler::GetInstance().CancelAll();
	log->info("TeleopInit...");
	teleopInit();
}

void EctoRobot::TeleopPeriodic() {
	inputHandlers->updateRobot();
	teleopUpdate();
}

void EctoRobot::DisabledInit() {
	log->info("DisabledInit...");
	systemManager->initDisabled();
}

void EctoRobot::DisabledPeriodic() {
	this->updateDisabled();
	systemManager->updateDisabled();
}

//TODO Add test runner
void EctoRobot::TestInit() {
	log->info("TestInit...");
}

void EctoRobot::TestPeriodic() {
	;
}

void EctoRobot::autoInit() {
	;
}

void EctoRobot::autoUpdate() {
	;
}

void EctoRobot::teleopInit() {
	;
}

void EctoRobot::teleopUpdate() {
	;
}

