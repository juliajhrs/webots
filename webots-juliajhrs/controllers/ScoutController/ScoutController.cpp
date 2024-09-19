// File:          ScoutController.cpp
// Date:          18/11/2023
// Description:   Implementation of Scout Robot that inherits BaseRobot class
// Author:        JULIA JOHARIS
// zID:           z5383753
// Modifications:

#include "ScoutController.hpp"

ScoutRobot::ScoutRobot() {
    // assigning sensors and actuators
    camera = getCamera("camera");
    leftMotor = getMotor("left wheel motor");
    rightMotor = getMotor("right wheel motor");
    camera->enable(TIME_STEP);
    camera->recognitionEnable(TIME_STEP);
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
}

ScoutRobot::~ScoutRobot() {
}

void ScoutRobot::run() {
    while (step(TIME_STEP) != -1) {
        auto data = receiveMessage();
        if (!(data.first.empty() && data.second.empty())) {
          std::istringstream issX(data.first);
          std::istringstream issY(data.second);
          issX >> targetPositionX;
          issY >> targetPositionY;
        }
        if (!(targetPositionX == 0 && targetPositionY == 0)) {
          // received coordinates, start moving
          updateCurrentPosition();
          if (setDirectionToTarget() == true) {
            moveToTarget(0.5);
          }
          // scans the OOI only after it stops
          if ((leftMotor->getVelocity() == 0) && (rightMotor->getVelocity() == 0)) {
            readColour();
            // scanned an OOI, reporting to leader
            sendMessage(
                std::to_string(LEADER_ID),  // const std::string& toID
                ID,                         // const std::string& data0
                std::to_string(seenObject)  // const std::string& data1
            );
          }
        }
    };
}

void ScoutRobot::move(double speed) {
    leftMotor->setVelocity(speed);
    rightMotor->setVelocity(speed);
}

void ScoutRobot::rotate(double speed) {
    leftMotor->setVelocity(-speed);
    rightMotor->setVelocity(speed);
}

bool ScoutRobot::readColour() {
  seenObject = camera->getRecognitionNumberOfObjects();
  if (seenObject == 1) {
    return true;
  }
  return false;
}

// RUN SIMULATION IN MAIN LOOP
int main(int argc, char **argv) {
    std::unique_ptr<ScoutRobot> scout{ std::make_unique<ScoutRobot>() };
    scout->run();
}