// File:          ScoutController.hpp
// Date:          18/11/2023
// Description:   Header file of Scout Robot that inherits BaseRobot class
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
#include <array>

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>

constexpr int LEADER_ID { 0 };

class ScoutRobot : public BaseRobot {
public:
	ScoutRobot();
	virtual ~ScoutRobot();

	virtual void run() override;
	virtual void move(double speed) override;
	virtual void rotate(double speed) override;

	bool readColour();
private:
	// sensors and actuators
	webots::Camera *camera{};
	webots::Motor *leftMotor{};
	webots::Motor *rightMotor{};

	// method data members
	int seenObject{};
};
