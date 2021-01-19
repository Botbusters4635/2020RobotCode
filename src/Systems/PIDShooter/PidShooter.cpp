//
// Created by abiel on 1/5/20.
//

#include <frc/Timer.h>
#include "PidShooter.h"

PIDShooter::PIDShooter(const PIDShooterConfig &config) : System("PIDShooter") {
	this->config = config;
	this->motor = config.motor;
	
	//motor->prioritizeUpdateRate();
	
	frc::TrapezoidProfile<units::radians>::Constraints constraints{
			units::radians_per_second_t(config.maximumAcceleration),
			units::unit_t<radians_per_second_squared_t>(config.maximumJerk)};
	
	frc::TrapezoidProfile<units::radians>::State goal;
	
	goal.position = units::radian_t(0);
	goal.velocity = units::radians_per_second_t(0);
	
	frc::TrapezoidProfile<units::radians>::State state;
	
	state.position = units::radian_t(0);
	state.velocity = units::radians_per_second_t(0);
	
	trapezoidProfile = std::make_unique<frc::TrapezoidProfile<units::radians >>(constraints, goal, state);
	
	voltageFilter = std::make_unique<frc::LinearDigitalFilter>(
			frc::LinearDigitalFilter::MovingAverage(voltageSource, voltageAccumulatorWindowSize));
	
	currentStage = PIDShooterStage::Stopped;
	currentVelocityTarget = 0;
	
	lastTime = 0.01;
	lastVelocity = 0.01;
	lastAcceleration = 0.01;
}

void PIDShooter::spinup(double radPerSecond) {
	if (radPerSecond == previousSpinUp) {
		return;
	}
	
	previousSpinUp = radPerSecond;
	
	currentStage = PIDShooterStage::SpinUp;
	currentVelocityTarget = radPerSecond * config.gearReduction;
	
	frc::TrapezoidProfile<units::radians>::Constraints constraints(
			units::radians_per_second_t(config.maximumAcceleration),
			units::unit_t<radians_per_second_squared_t>(config.maximumJerk));
	
	frc::TrapezoidProfile<units::radians>::State goal;
	
	goal.position = units::radian_t(currentVelocityTarget);
	goal.velocity = units::radians_per_second_t(config.maximumAcceleration);
	
	frc::TrapezoidProfile<units::radians>::State state;
	
	state.position = units::radian_t(motor->getVelocity());
	state.velocity = units::radians_per_second_t(0);
	
	trapezoidProfile = std::make_unique<frc::TrapezoidProfile<units::radians >>(constraints, goal, state);
	trapezoidalProfileStartTime = frc::Timer::GetFPGATimestamp();
}

bool PIDShooter::isReadyToShoot() const {
	return std::abs(motor->getVelocity() - currentVelocityTarget) <= config.velocityTolerance;
	//return stableSamples >= config.samplesUntilReadyToShoot;
}

void PIDShooter::enableOpenLoop(bool state) {
	holdOpen = state;
}

void PIDShooter::setPIDConfig(const PIDConfig &config) {
	motor->setPIDConfig(config, 0);
	this->config.pidConfig = config;
}

void PIDShooter::initRobot() {
	motor->setControlMode(MotorControlMode::Velocity);
	motor->setPIDConfig(config.pidConfig, 0);
	motor->setClosedLoopRampRate(config.motorClosedLoopRampRate);
	motor->setOpenLoopRampRate(config.motorOpenLoopRampRate);
	
	motor->setMotorCurrentLimit(config.motorCurrentLimit);
	motor->enableCurrentLimit(true);
	
	table = ntInstance.GetTable("PIDShooter");
	
	table->GetEntry("PID/P").SetDouble(config.pidConfig.p);
	table->GetEntry("PID/I").SetDouble(config.pidConfig.i);
	table->GetEntry("PID/D").SetDouble(config.pidConfig.d);
	table->GetEntry("PID/F").SetDouble(config.pidConfig.f);
	
	networkTableUpdate();
}

void PIDShooter::updateRobot() {
	networkTableUpdate();
	
	const double dt = frc::Timer::GetFPGATimestamp() - lastTime;
	
	if (isReadyToShoot()) {
		PatternCommand command;
		command.primaryPattern = LEDPattern::RedFire;
		
		manager.queueCommand(command, PatternPriority::MedPriority);
	} else {
		PatternCommand command;
		command.primaryPattern = LEDPattern::GreenFire;
		
		manager.queueCommand(command, PatternPriority::MedPriority);
	}
	
	velocity = getVelocity();
	if (velocity != lastVelocity) {
		const double velocityDt = frc::Timer::GetFPGATimestamp() - lastVelocityTime;
		
		acceleration = (velocity - lastVelocity) / velocityDt;
		jerk = (acceleration - lastAcceleration) / velocityDt;
		
		lastVelocity = velocity;
		lastAcceleration = acceleration;
		
		lastVelocityTime = frc::Timer::GetFPGATimestamp();
	}
	
	switch (currentStage) {
		case PIDShooterStage::Stopped:
			motor->setControlMode(MotorControlMode::Percent);
			motor->set(0);
			break;
		
		case PIDShooterStage::SpinUp:
			motor->setControlMode(MotorControlMode::Velocity);
			//lastMotorVelocitySetPoint = trapezoidProfile->Calculate(
			//		units::second_t(
			//				frc::Timer::GetFPGATimestamp() - trapezoidalProfileStartTime)).position.to<double>();
			motor->set(currentVelocityTarget);
			
			if (std::abs(velocity - currentVelocityTarget) <= config.velocityTolerance) {
				//Target velocity has been reached
				//currentStage = PIDShooterStage::HoldPID;
				break;
			}
			break;
		
		case PIDShooterStage::HoldPID:
			/**
			 * Reset timer if the velocity has not been held stable
			 */
			if (!hasHeldVelocityStable) {
				heldVelocityStartTime = frc::Timer::GetFPGATimestamp();
				hasHeldVelocityStable = true;
				break;
			}
			
			if (std::abs(velocity - currentVelocityTarget) > config.velocityTolerance) {
				//Motor is outside of velocity range
				hasHeldVelocityStable = false;
			}
			
			/**
			 * Check if the motor has held the velocity for a certain time
			 */
			if (hasHeldVelocityStable and
			    frc::Timer::GetFPGATimestamp() - heldVelocityStartTime > config.timeToWaitToStabilize) {
				//Velocity is stable get voltage samples
				currentStage = PIDShooterStage::CollectSamples;
				stableSamples = 0;
				voltageFilter->Reset();
				break;
			}
			
			break;
		
		case PIDShooterStage::CollectSamples:
			/**
			 * Keep stable until ball enters the shooter
			 */
			voltageSource.set(motor->getPercentOutput());
			voltageFilter.get();
			stableSamples++;
			
			if (isReadyToShoot() and holdOpen) {
				currentStage = PIDShooterStage::HoldOpen;
			}
			break;
		
		case PIDShooterStage::HoldOpen:
			motor->setControlMode(MotorControlMode::Percent);
			motor->set(voltageFilter->Get());
			holdOpen = false;
			
			break;
	}
	
	lastTime = frc::Timer::GetFPGATimestamp();
}

std::string PIDShooter::shooterStageToString(PIDShooterStage stage) {
	switch (stage) {
		case PIDShooterStage::Stopped:
			return ("Stopped");
		
		case PIDShooterStage::SpinUp:
			return ("SpinUp");
		
		case PIDShooterStage::HoldPID:
			return ("HoldPID");
		
		case PIDShooterStage::CollectSamples:
			return ("HoldStable");
		
		case PIDShooterStage::HoldOpen:
			return ("HoldOpen");
		
		default:
			return ("InvalidEnum");
	}
}

double PIDShooter::getVelocity() const {
	return (motor->getVelocity() / config.gearReduction);
}

double PIDShooter::getAcceleration() const {
	return acceleration;
}

double PIDShooter::getJerk() const {
	return jerk;
}

void PIDShooter::networkTableUpdate() {
	table->GetEntry("LastMotorSetPoint").SetDouble(lastMotorVelocitySetPoint);
	table->GetEntry("MotorVelocityError").SetDouble(motor->getVelocity() - lastMotorVelocitySetPoint);
	
	table->GetEntry("RawVelocity").SetDouble(motor->getVelocity());
	table->GetEntry("Velocity").SetDouble(getVelocity());
	table->GetEntry("Acceleration").SetDouble(getAcceleration());
	table->GetEntry("Jerk").SetDouble(getJerk());
	
	table->GetEntry("CurrentStage").SetString(PIDShooter::shooterStageToString(currentStage));
	table->GetEntry("StableSamples").SetDouble(stableSamples);
	table->GetEntry("HasHeldVelocityStable").SetBoolean(hasHeldVelocityStable);
	table->GetEntry("HoldOpen").SetBoolean(holdOpen);
	table->GetEntry("CurrentVelocityTarget").SetDouble(currentVelocityTarget);
	table->GetEntry("IsReadyToShoot").SetBoolean(isReadyToShoot());
	
	PIDConfig newPid = config.pidConfig;
	newPid.p = table->GetEntry("PID/P").GetDouble(config.pidConfig.p);
	newPid.i = table->GetEntry("PID/I").GetDouble(config.pidConfig.i);
	newPid.d = table->GetEntry("PID/D").GetDouble(config.pidConfig.d);
	newPid.f = table->GetEntry("PID/F").GetDouble(config.pidConfig.f);
	
	if (newPid != config.pidConfig) {
		setPIDConfig(newPid);
	}
}

void PIDShooter::setHood(HoodPosition position) {
	pcm.setPistonState(pcm.getPiston("hood"), position == HoodPosition::CloseRange);
}

