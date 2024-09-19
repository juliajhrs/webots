// File:          BaseRobot.hpp
// Date:          17/11/2023
// Description:   Header file of BaseRobot to be inherited by the Leader and Scout robots classes.
// Author:        JULIA JOHARIS
// zID:           z5383753
// Modifications:

#pragma once

// add additional includes as needed
#include <iostream>
#include <memory>
#include <sstream>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>

#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>


constexpr int TIME_STEP{ 64 };
constexpr double maxEpuckMotorSpeed{ 6.28 };
constexpr double PI{ 3.14159265358979323846 };

class BaseRobot : public webots::Robot {
public:
	BaseRobot();
	virtual ~BaseRobot();
  
	virtual void run() = 0;
	virtual void move(double speed) = 0;
	virtual void rotate(double speed) = 0;

	void keyboardControl();
	void updateCurrentPosition();
	double getBearingInDegrees();
	void setTargetPosition(double x, double y);
	bool setDirectionToTarget();
	bool moveToTarget(double stopDistance);
	   
	void sendMessage(const std::string& ID, const std::string& data0, const std::string& data1);
	std::pair<std::string, std::string> receiveMessage();
protected:
	std::string ID{};
	double currentPositionX{};
	double currentPositionY{};
	double currentYaw{};
	double targetPositionX{};
	double targetPositionY{};
	double angle{};
	
           webots::Receiver *receiver{};
	webots::Emitter *emitter{};
	webots::GPS *gps{};
	webots::Compass *compass{};
	webots::Keyboard *keyboard{};
	
	struct OOI {
		double x{};
		double y{};
	};
	std::vector<OOI> OOICoordinates{};
private:
	// add additional members as needed
};
