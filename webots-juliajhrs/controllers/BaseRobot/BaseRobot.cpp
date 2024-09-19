// File:          BaseRobot.cpp
// Date:          17/11/2023
// Description:   Implementation of BaseRobot to be inherited by the Leader and Scout robots classes.
// Author:        JULIA JOHARIS
// zID:           z5383753
// Modifications:

#include "BaseRobot.hpp"

BaseRobot::BaseRobot() {
    // assigning sensors and actuators
    ID = getName();
    receiver = getReceiver("receiver");
    emitter = getEmitter("emitter");
    gps = getGPS("gps");
    compass = getCompass("compass");
    // enabling sensors and actuators
    receiver->enable(TIME_STEP);
    gps->enable(TIME_STEP);
    compass->enable(TIME_STEP);
}

BaseRobot::~BaseRobot() {
}

void BaseRobot::keyboardControl() {
    while (step(TIME_STEP) != -1) {
        char key = static_cast<char>(keyboard->getKey());
        switch (key) {
        case 'W':
            move(maxEpuckMotorSpeed);
            break;
        case 'A':
            // goes left
            rotate(maxEpuckMotorSpeed);
            break;
        case 'S':
            // moves backward
            move(-maxEpuckMotorSpeed);
            break;
        case 'D':
            // goes right
            rotate(-maxEpuckMotorSpeed);
            break;
        case ' ':
            rotate(maxEpuckMotorSpeed);
            break;
        }
    }
}

void BaseRobot::updateCurrentPosition() {
    const double *gpsValues = gps->getValues();
    currentPositionX = gpsValues[0];
    currentPositionY = gpsValues[1];
    currentYaw = getBearingInDegrees();
}

double BaseRobot::getBearingInDegrees() {
    const double *north = compass->getValues();
    double rad = atan2(north[1], north[0]);
    double bearing = (rad - 1.5708) / PI * 180.0;
    if (bearing < 0.0)
      bearing = bearing + 360.0;
    return bearing;
}

void BaseRobot::setTargetPosition(double x, double y) {
    targetPositionX = x;
    targetPositionY = y;
}

bool BaseRobot::setDirectionToTarget() {
    // ensures ogCurrents does not update to avoid spinning every time step
    static double ogCurrentX;
    static double ogCurrentY;
    static bool isFirstIteration = false;
    if (!isFirstIteration) {
        ogCurrentX = currentPositionX;
        ogCurrentY = currentPositionY;
        isFirstIteration = true; // updates the flag after setting the initial position
    }
    int offset;
    double deltaX = targetPositionX - ogCurrentX;
    double deltaY = targetPositionX - ogCurrentY;
    double rad = atan2(deltaY, deltaX);
    angle = rad/(4 * atan2(1, 1)) * 180.0; // converts to degrees
    if (ID.compare("1") == 0) {
      offset = 10;
      angle = (angle < 0.0) ? (360.0 + angle) - offset: angle - offset;
    } else if (ID.compare("2") == 0) {
      offset = 260;
      angle = (angle < 0.0) ? (360.0 + angle) + offset: angle + offset;
    } else if (ID.compare("3") == 0){
      offset = 10;
      angle = (angle < 0.0) ? (360.0 + angle) - offset: angle - offset;
    } else if (ID.compare("0") == 0) {
      offset = 260;
      angle = (angle < 0.0) ? (360.0 + angle) + offset: angle + offset;
    }
    //updateCurrentPosition();
    double threshold = 0.5;
    if (std::abs(currentYaw - angle) > threshold) {
      rotate(0.5);
      return false;
    }
    // if theres no diff between currentYaw and angle, means its facing it == stop rotating
    rotate(0);
    return true;
}

bool BaseRobot::moveToTarget(double stopDistance) {
      double distance = std::sqrt(
        std::pow(targetPositionX - currentPositionX, 2) +
        std::pow(targetPositionY - currentPositionY, 2)
      );
      if (distance > stopDistance) {
        // robot is not at the target point, keep moving
        move(2);
        //setDirectionToTarget();
        return false;
      }
      move(0);
      return true;
}

void BaseRobot::sendMessage(const std::string& ID, const std::string& data0, const std::string& data1) {
    std::cout << "Sending message to " << ID << std::endl;
    std::string message{};
    message.append(ID);
    message.append("|");
    message.append(data0);
    message.append("|");
    message.append(data1);
    emitter->send(message.c_str(), (int)strlen(message.c_str()) + 1);
}


std::pair<std::string, std::string> BaseRobot::receiveMessage() {
    if (receiver->getQueueLength() > 0) {
        std::string message{ static_cast<const char*>(receiver->getData()) };
        receiver->nextPacket();

        std::istringstream iss{ message };
        std::string incomingId{};
        std::getline(iss, incomingId, '|');

        if (ID.compare(incomingId) == 0) {
            // ID matches, now extract data0 and data1
            std::string data0{};
            std::string data1{};
            if (std::getline(iss, data0, '|') && std::getline(iss, data1, '|')) {
                std::cout << "Received message with matching ID: " << message << std::endl;
                return std::make_pair(data0, data1);
            }
        }
    }
    // If the ID doesn't match or the format is incorrect, return an empty pair
    return std::make_pair("", "");
}