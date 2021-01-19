//
// Created by abiel on 1/5/20.
//

#include "Droideka.h"
#include <Control/Path/PathParserWPILIB.h>
#include <frc/trajectory/TrajectoryUtil.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/Compressor.h>

Droideka::Droideka() : EctoRobot("Droideka") {
	frc::Compressor compressor(1);
	compressor.Start();
	elevator = std::make_shared<Elevator>(MotorManager::getInstance().getMotor("elevator"));
	ElevatorInputHandlerConfig elevatorInputHandlerConfig;
	elevatorInputHandlerConfig.elevator = elevator;
	
	elevatorInputHandler = std::make_shared<ElevatorInputHandler>(elevatorInputHandlerConfig);
	
	EctoSwerveConfig swerveConfig;
	
	swerve = std::make_shared<EctoSwerve>(swerveConfig);
	swerveInputHandler = std::make_shared<EctoSwerveInputHandler>(*(swerve.get()));
	
	leftFlywheel = MotorManager::getInstance().getMotor("flywheel_left");
	rightFlywheel = MotorManager::getInstance().getMotor("flywheel_right");
	
	/**
	 * Shooter setup
	 */
	rightFlywheel->followMotor(*leftFlywheel, true);
	rightFlywheel->enableBrakingOnIdle(false);
	rightFlywheel->enableCurrentLimit(true);
	rightFlywheel->setMotorCurrentLimit(30);
	leftFlywheel->setMotorCurrentLimit(30);
	leftFlywheel->enableCurrentLimit(true);
	leftFlywheel->enableBrakingOnIdle(false);
	
	PIDShooterConfig shooterConfig;
	shooterConfig.maximumAcceleration = 260;
	shooterConfig.motor = leftFlywheel;
	shooterConfig.pidConfig.p = 0.0015;
	shooterConfig.pidConfig.f = 0.0002;
	
	shooter = std::make_shared<PIDShooter>(shooterConfig);
	
	/**
	 * Ball Feeder setup
	 */
	BallFeederConfig feederConfig;
	feederConfig.motor = handler.getMotor("indexer");
	ballFeeder = std::make_shared<BallFeeder>(feederConfig);
	
	/**
	 * Ball intake setup
	 */
	ballIntake = std::make_shared<BallIntake>(handler.getMotor("left_intake"), handler.getMotor("right_intake"));
	
	/**
	 * Ball input handler
	 */
	BallInputHandlerConfig ballInputHandlerConfig;
	ballInputHandlerConfig.feeder = ballFeeder;
	ballInputHandlerConfig.intake = ballIntake;
	ballInputHandlerConfig.shooter = shooter;
	ballInputHandler = std::make_shared<BallInputHandler>(ballInputHandlerConfig);
	
	systemManager->addSubsystem(ballFeeder);
	systemManager->addSubsystem(ballIntake);
	inputHandlers->addSubsystem(ballInputHandler);
	
	systemManager->addSubsystem(swerve);
	inputHandlers->addSubsystem(swerveInputHandler);
	
	systemManager->addSubsystem(elevator);
	inputHandlers->addSubsystem(elevatorInputHandler);
	
	systemManager->addSubsystem(shooter);
	
	ShooterCharacterizerConfig characterizerConfig;
	characterizerConfig.leftMotor = leftFlywheel;
	characterizerConfig.rightMotor = rightFlywheel;
	
	
	//characterizer = std::make_shared<ShooterCharacterizer>(characterizerConfig);
	//systemManager->addSubsystem(characterizer);
	trajConfig.positionPIDConfig.p = 4.50;
	trajConfig.positionPIDConfig.i = 0.0;
	trajConfig.positionPIDConfig.d = 0.0001;
	
	trajConfig.thetaPIDConfig.p = 2.5;
	trajConfig.thetaPIDConfig.i = 0;
	trajConfig.thetaPIDConfig.f = 0;
	
	trajConfig.maximumVelocity = 1.25;
	trajConfig.maximumAngularVelocity = 1.0;
	trajConfig.angularVelocityReductionFactor = 0.55;
	
	trajectory = frc::TrajectoryUtil::FromPathweaverJson(
			Module::getConfigFileRootDir() + "Paths/output/ShootFirst3.wpilib.json");
	
	trajectory2 = frc::TrajectoryUtil::FromPathweaverJson(
			Module::getConfigFileRootDir() + "Paths/output/IntakeBalls.wpilib.json");
	
	trajectory3 = frc::TrajectoryUtil::FromPathweaverJson(
			Module::getConfigFileRootDir() + "Paths/output/ShootLast2.wpilib.json");
	
	LowerIntakeSetpoint lowerIntakeSetpoint;
	lowerIntakeSetpoint.lowerLeft = true;
	lowerIntakeSetpoint.lowerRight = false;
	lowerIntakeSetpoint.leftSetpoint = 1.0;
	lowerIntakeSetpoint.rightSetpoint = 0.0;
	
	LowerIntakeSetpoint lowerIntakeStop;
	lowerIntakeStop.lowerLeft = false;
	lowerIntakeStop.lowerRight = false;
	lowerIntakeStop.leftSetpoint = 0.0;
	lowerIntakeStop.rightSetpoint = 0.0;

//	shootBalls = std::make_shared<frc2::SequentialCommandGroup>(
//			frc2::ParallelCommandGroup{WPITrajectoryFollower{swerve, trajConfig, trajectory, M_PI},
//			                           SpinupFlywheel{shooter, 300}}, FeedBalls{ballFeeder, 3},
//			frc2::ParallelCommandGroup{WPITrajectoryFollower{swerve, trajConfig, trajectory2, M_PI},
//			                           LowerIntake{ballIntake, lowerIntakeSetpoint}},
//			LowerIntake{ballIntake, lowerIntakeStop},
//			frc2::SequentialCommandGroup{WPITrajectoryFollower{swerve, trajConfig, trajectory3, M_PI}, FeedBalls{ballFeeder, 2}});
	
	PatternCommand testCommand;
	testCommand.primaryPattern = LEDPattern::BlueFire;
	testCommand.secondaryPattern = LEDPattern::RandomBlink;
	
	testCommand.primaryColor = {0, 255, 0};
	testCommand.secondaryColor = {0, 0, 255};
	testCommand.patternRate = .01;
	
	ledManager.queueCommand(testCommand, PatternPriority::LowPriority);
	
	visionManager.setCameraLEDRingState(false);
	
	pathHelper = std::make_unique<TeleopPathHelper>(swerve);
	systemManager->addSubsystem(pathHelper);
	
	// errorTester = std::make_shared<SwerveErrorTester>(swerve);
	// systemManager->addSubsystem(errorTester);
}

void Droideka::autoInit() {
	swerve->updateInitialPositionChooser();
	
	frc2::CommandScheduler::GetInstance().OnCommandInitialize(
			[](const frc2::Command &command) {
				frc::SmartDashboard::PutString(
						"Command Initialized", command.GetName());
			}
	);
	
	frc2::CommandScheduler::GetInstance().OnCommandInterrupt(
			[](const frc2::Command &command) {
				frc::SmartDashboard::PutString(
						"Command Interrupted", command.GetName());
			}
	);
	
	frc2::CommandScheduler::GetInstance().OnCommandFinish(
			[](const frc2::Command &command) {
				frc::SmartDashboard::PutString(
						"Command Finished", command.GetName());
			}
	);
	
	basicTestAuto = std::make_shared<BasicAuto>(swerve, ballFeeder, ballIntake, shooter);
	basicTestAuto->Schedule();
	
	//shootBalls->Schedule();
	//testPath->Schedule();
}

void Droideka::autoUpdate() {
	//shooter->setTargetVelocity(ntInstance.GetTable("StateSpaceShooter")->GetEntry("TargetVelocityTarget").GetDouble(0));
}

void Droideka::teleopInit() {
	ntInstance.GetTable("StateSpaceShooter")->GetEntry("TargetVelocityTarget").SetDouble(0);
	ntInstance.GetTable("Elevator")->GetEntry("Motor").SetDouble(0);
}

void Droideka::teleopUpdate() {
	//handler.getMotor("elevator")->set(ntInstance.GetTable("Elevator")->GetEntry("Motor").GetDouble(0));
	//handler.getMotor("elevator")->set(1.0); //Positive = up
	//ntInstance.GetTable("DistanceSensors")->GetEntry("Right").SetDouble(rightInput.GetAverageVoltage());
	//ntInstance.GetTable("DistanceSensors")->GetEntry("Left").SetDouble(leftInput.GetAverageVoltage());
	ntInstance.GetTable("StateSpaceShooter")->GetEntry("LeftCurrent").SetDouble(leftFlywheel->getMotorCurrent());
	ntInstance.GetTable("StateSpaceShooter")->GetEntry("RightCurrent").SetDouble(rightFlywheel->getMotorCurrent());
	//shooter->spinup(ntInstance.GetTable("StateSpaceShooter")->GetEntry("TargetVelocityTarget").GetDouble(0));
}