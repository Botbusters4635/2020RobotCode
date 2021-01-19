//
// Created by abiel on 2/12/20.
//

#include "VisionManager.h"
#include <frc/Timer.h>
#include <Math/EctoMath.h>

VisionManager::VisionManager() : Manager<VisionManager>("VisionManager") {
	jetsonTable = ntInstance.GetTable(visionManagerBaseKey);
	roborioTable = ntInstance.GetTable(visionManagerBaseKeyRoborio);
	
	timeSync.RequestInterrupts(std::bind(&VisionManager::timeSyncDigitalInterrupt, this, std::placeholders::_1));
	timeSync.EnableInterrupts();
	
	jetsonTable->GetEntry(timestampKey).AddListener(
			std::bind(&VisionManager::timeSyncNetworkInterrupt, this, std::placeholders::_1), NT_NOTIFY_UPDATE);
	jetsonTable->GetEntry(dataKey).AddListener(
			std::bind(&VisionManager::dataKeyNetworkInterrupt, this, std::placeholders::_1), NT_NOTIFY_UPDATE);
}

void VisionManager::timeSyncNetworkInterrupt(const nt::EntryNotification &event) {
	lastTimeSyncNetworkPulse_jetson.store(event.value->GetDouble());
	lastTimeSyncNetworkPulse_roborio.store(frc::Timer::GetFPGATimestamp());
}

void VisionManager::timeSyncDigitalInterrupt(frc::InterruptableSensorBase::WaitResult waitResult) {
	lastTimeSyncDigitalPulse_roborio.store(frc::Timer::GetFPGATimestamp());
}

void VisionManager::dataKeyNetworkInterrupt(const nt::EntryNotification &event) {
	std::lock_guard<std::mutex> lockGuard(latestRawMessageLock);
	latestRawMessage = event.value->GetDoubleArray();
}

double VisionManager::transformJetsonTimeToRoborio(double jetsonTime) const {
	return (jetsonTime - networkDelta) - jetsonDelta;
}

void VisionManager::updateNetworkTables() {
	roborioTable->GetEntry("LastTimeSyncNetwork/Jetson").SetDouble(lastTimeSyncNetworkPulse_jetson.load());
	roborioTable->GetEntry("LastTimeSyncNetwork/RoboRio").SetDouble(lastTimeSyncNetworkPulse_roborio.load());
	
	roborioTable->GetEntry("LastTimeDigital/RoboRio").SetDouble(lastTimeSyncDigitalPulse_roborio.load());
}

VisionDataContainer VisionManager::parseRawData(const std::string &msg) {
	VisionDataContainer message;
	std::stringstream in(msg);
	
	std::vector<std::string> readRows;
	
	std::string cache;
	while (std::getline(in, cache, ',')) {
		readRows.emplace_back(cache);
	}
	
	if (readRows.size() != 10) {
		log->error("Read invalid message from jetson: {}", msg);
	}
	
	const double x = xFilter.Calculate(std::stod(readRows.at(3)));
	const double y = yFilter.Calculate(std::stod(readRows.at(5)));
	message.angle0 = std::stod(readRows.at(6));
	const Rotation2D heading(thetaFilter.Calculate(std::stod(readRows.at(2))));
	
	
	message.robotPose = RobotPose2D(x, y, heading);
	
	message.angle1 = std::stod(readRows.at(7));
	
	message.jetsonTimestamp = std::stod(readRows.at(9));
	message.roborioTimestamp = VisionManager::transformJetsonTimeToRoborio(message.jetsonTimestamp);
	message.isDetected = readRows.at(8) == "True";
	
	return message;
}

VisionDataContainer VisionManager::getLatestData() const {
	return latestData;
}

RobotPose2D VisionManager::correctDataForLatency(const VisionDataContainer &data,
                                                 const Twist2D &currentVelocity) {
	const double dt = frc::Timer::GetFPGATimestamp() - data.roborioTimestamp;
	double dx, dy, dtheta;
	dx = currentVelocity.getDx() * dt;
	dy = currentVelocity.getDy() * dt;
	dtheta = currentVelocity.getDtheta() * dt;
	return {dx + data.robotPose.getX(), dy + data.robotPose.getY(),
	        EctoMath::wrapAngle(dtheta + data.robotPose.getHeading().getRadians())};
}

void VisionManager::update() {
	const double currentTime = frc::Timer::GetFPGATimestamp();
	
	updateNetworkTables();
	
	latestRawMessageLock.lock();
	const std::vector<double> rawMessage = latestRawMessage;
	latestRawMessageLock.unlock();
	
	if (!rawMessage.empty()) {
		//Parse raw message
		latestData.robotPose.setX(xFilter.Calculate(rawMessage.at(0)));
		latestData.robotPose.setY(yFilter.Calculate(rawMessage.at(1)));
		
		latestData.robotPose.setHeading(Rotation2D(
				thetaFilter.Calculate((jetsonTable->GetEntry("VisionCamera/TargetYaw").GetDouble(0) / 180.0) * M_PI)));
		
		latestData.isDetected = jetsonTable->GetEntry("isValid").GetBoolean(false);
	}
	
	if (currentTime - lastTimeSyncCheck > timeSyncCheckInterval) {
		const double roborioNetworkPulse = lastTimeSyncNetworkPulse_roborio.load();
		const double jetsonNetworkPulse = lastTimeSyncNetworkPulse_jetson.load();
		const double roborioDigitalPulse = lastTimeSyncDigitalPulse_roborio.load();
		
		//Calculate time difference between digital and network interrupts
		networkDelta = roborioNetworkPulse - roborioDigitalPulse;
		jetsonDelta = (jetsonNetworkPulse - networkDelta) - roborioDigitalPulse;
		
		lastTimeSyncCheck = currentTime;
	}
}

void VisionManager::setCameraLEDRingState(bool state) {
	ledRelay.Set(!state);
}