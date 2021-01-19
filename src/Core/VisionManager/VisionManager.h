//
// Created by abiel on 2/12/20.
//

#ifndef BOTBUSTERSREBIRTH_VISIONMANAGER_H
#define BOTBUSTERSREBIRTH_VISIONMANAGER_H

#include <Core/EctoModule/Manager.h>
#include <Math/DataTypes/RobotPose2D.h>

#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/MedianFilter.h>

#include <Math/DataTypes/Twist2D.h>

struct VisionDataContainer {
	RobotPose2D robotPose{0, 0, 0};
	double angle0, angle1;
	
	double jetsonTimestamp{0};
	double roborioTimestamp{0};
	
	bool isDetected{false};
};

class VisionManager : public Manager<VisionManager> {
	friend class Manager<VisionManager>;

public:
	VisionDataContainer getLatestData() const;
	
	void setCameraLEDRingState(bool state);
	
	/**
	* Twist2D uses field oriented velocities
	* @param data
	* @param currentVelocity
	* @return
	*/
	static RobotPose2D correctDataForLatency(const VisionDataContainer &data, const Twist2D &currentVelocity);

protected:
	void update() override;

private:
	VisionManager();
	
	void updateNetworkTables();
	
	VisionManager &operator=(const VisionManager &);
	
	frc::DigitalOutput ledRelay{0};
	
	const size_t medianFilterSamples = 1;
	frc::MedianFilter<double> xFilter{medianFilterSamples};
	frc::MedianFilter<double> yFilter{medianFilterSamples};
	frc::MedianFilter<double> thetaFilter{medianFilterSamples};
	
	const std::string visionManagerBaseKey = "chameleon-vision/VisionCamera";
	const std::string visionManagerBaseKeyRoborio = "EctoVisionRoborio";
	const std::string timestampKey = "timestamp";
	const std::string dataKey = "targetPose";
	
	double transformJetsonTimeToRoborio(double jetsonTime) const;
	
	VisionDataContainer parseRawData(const std::string &msg);
	
	const int timeSyncChannel = 1;
	
	std::atomic<double> lastTimeSyncNetworkPulse_jetson;
	std::atomic<double> lastTimeSyncNetworkPulse_roborio;
	
	std::atomic<double> lastTimeSyncDigitalPulse_roborio;
	double networkDelta{0};
	double jetsonDelta{0};
	
	std::mutex latestRawMessageLock;
	std::vector<double> latestRawMessage;
	VisionDataContainer latestData;
	
	void timeSyncDigitalInterrupt(frc::InterruptableSensorBase::WaitResult waitResult);
	
	void timeSyncNetworkInterrupt(const nt::EntryNotification &event);
	
	void dataKeyNetworkInterrupt(const nt::EntryNotification &event);
	
	const double timeSyncCheckInterval = 5;
	double lastTimeSyncCheck;
	
	frc::DigitalInput timeSync{timeSyncChannel};
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> jetsonTable, roborioTable;
};


#endif //BOTBUSTERSREBIRTH_VISIONMANAGER_H
