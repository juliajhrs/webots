// File:          LeaderController.hpp
// Date:          18/11/2023
// Description:   Header file of Leader Robot that inherits BaseRobot class
// Author:        JULIA JOHARIS
// zID:           z5383753
// Modifications:

#pragma once

// add additional includes as needed
#include "BaseRobot.hpp"

#include <iostream>
#include <memory>
#include <sstream>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <limits>

#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>

class LeaderRobot : public BaseRobot {
public:
	LeaderRobot();
	virtual ~LeaderRobot();

	void run() override;
	void move(double speed) override;
	void rotate(double speed) override;
	
	void scanLidarData();
	void printLidarData();
	void assignToScouts();
	void fileOutput(const std::string& output);
private:
	// data member structs
	struct PointCoordinates {
		float x{};
		float y{};
	};
	
	// sensors and actuators
	webots::Lidar *mLidar{};
	webots::Motor* frontLeftMotor{};
	webots::Motor* frontRightMotor{};
	webots::Motor* rearLeftMotor{};
	webots::Motor* rearRightMotor{};
	webots::Keyboard *keyboard{};
	
	// method data members
	int keyboardCheck{};
	std::vector<PointCoordinates> pointCloud{};
	std::string datalog{};
};
