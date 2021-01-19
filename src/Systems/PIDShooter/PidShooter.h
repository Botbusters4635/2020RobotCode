//
// Created by abiel on 1/5/20.
//

#ifndef BOTBUSTERSREBIRTH_PIDSHOOTER_H
#define BOTBUSTERSREBIRTH_PIDSHOOTER_H

#include <Core/EctoModule/System.h>

#include <Control/EctoPID/EctoPIDAsynchronous.h>
#include <Control/MotionProfiles/TrapezoidalMotionProfile.h>

#include "Core/MotorHandler/EctoMotor/EctoMotor.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/filters/LinearDigitalFilter.h>
#include "Utilities/WPI/SimplePIDSource.h"

#include <frc/trajectory/TrapezoidProfile.h>
#include <Core/PCM/PCMManager.h>

#include "Core/LEDManager/LedManager.h"

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

/**
 * Using 254's control
 */

/**
 * The shooter subsystem consists of 4 775 Pro motors driving twin backspin flywheels. When run in reverse, these motors
 * power the robot's climber through a 1 way bearing. The shooter subsystem goes through 3 stages when shooting.
 * 1. Spin Up
 *  Use a PIDF controller to spin up to the desired RPM. We acquire this desired RPM by converting the camera's range
 *  value into an RPM value using the range map in the {@link Constants} class.
 * 2. Hold When Ready
 *  Once the flywheel's
 *  RPM stabilizes (remains within a certain bandwidth for certain amount of time), the shooter switches to the hold when
 *  ready stage. In this stage, we collect kF samples. The idea is that we want to run the shooter in open loop when we
 *  start firing, so in this stage we calculateOutput the voltage we need to supply to spin the flywheel at the desired RPM.
 * 3. Hold
 *  Once we collect enough kF samples, the shooter switches to the hold stage. This is the stage that we begin
 *  firing balls. We set kP, kI, and kD all to 0 and use the kF value we calculated in the previous stage for essentially
 *  open loop control. The reason we fire in open loop is that we found it creates a much narrower stream and leads to
 *  smaller RPM drops between fuel shots.
*/
struct PIDShooterConfig {
	PIDConfig pidConfig;
	
	double maximumVelocity = 1;
	double maximumAcceleration = 60;
	double maximumJerk = 100;
	
	double gearReduction = (15.0 / 25.0);
	
	std::shared_ptr<EctoMotor> motor;
	
	double velocityTolerance = 10;
	
	double timeToWaitToStabilize = 1;
	size_t samplesUntilReadyToShoot = 40;
	
	double motorCurrentLimit = 40;
	double motorClosedLoopRampRate = .1;
	double motorOpenLoopRampRate = .01;
};

enum class PIDShooterStage {
	Stopped,
	SpinUp,
	HoldPID,
	CollectSamples, //Velocity is stable for samples to be taken
	HoldOpen
};

enum class HoodPosition {
	LongRange,
	CloseRange
};

class PIDShooter : public System {
public:
	using radians_per_second_squared_t =
	units::compound_unit<units::radians,
			units::inverse<units::squared<units::second>>>;
	
	PIDShooter(const PIDShooterConfig &config);
	
	void initRobot() override;
	
	void updateRobot() override;
	
	/**
	 * Sets the shooter to open loop mode
	 */
	void enableOpenLoop(bool state);
	
	bool isReadyToShoot() const;
	
	void setHood(HoodPosition position);
	
	/**
	 * Spins up the flywheel and maintains velocity at a certain velocity
	 * @param radPerSecond
	 */
	void spinup(double radPerSecond);
	
	void setPIDConfig(const PIDConfig &config);
	
	double getVelocity() const;
	
	double getAcceleration() const;
	
	double getJerk() const;

private:
	static std::string shooterStageToString(PIDShooterStage stage);
	
	PCMManager &pcm = PCMManager::getInstance();
	LEDManager &manager = LEDManager::getInstance();
	
	std::shared_ptr<EctoMotor> motor;
	
	/**
	 * NT
	 */
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	void networkTableUpdate();
	
	PIDShooterConfig config;
	PIDShooterStage currentStage;
	
	double currentVelocityTarget;
	
	/**
	 * Velocity samples
	 */
	double heldVelocityStartTime;
	bool hasHeldVelocityStable = false;
	
	size_t stableSamples = 0;
	
	bool holdOpen = false;
	
	const size_t voltageAccumulatorWindowSize = 10;
	std::unique_ptr<frc::LinearDigitalFilter> voltageFilter;
	SimplePIDSource voltageSource;
	
	/**
	 * Velocity control
	 */
	//Using radians as radians/sec in order to avoid unit issues with radians / sec ^ 3 (jerk)
	std::unique_ptr<frc::TrapezoidProfile<units::radians>> trapezoidProfile;
	double trapezoidalProfileStartTime;
	
	double velocity, acceleration, jerk;
	double lastVelocityTime, lastVelocity, lastAcceleration;
	double lastTime;
	
	double lastMotorVelocitySetPoint;
	
	double previousSpinUp;
};


#endif //BOTBUSTERSREBIRTH_PIDSHOOTER_H
