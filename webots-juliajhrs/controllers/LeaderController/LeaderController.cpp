// File:          LeaderController.cpp
// Date:          18/11/2023
// Description:   Implementation of Leader Robot that inherits BaseRobot class
// Author:        JULIA JOHARIS
// zID:           z5383753
// Modifications:

#include "LeaderController.hpp"

LeaderRobot::LeaderRobot() {
    // assigning sensors and actuators
    mLidar = getLidar("lidar");
    frontLeftMotor = getMotor("front left wheel motor");
    frontRightMotor = getMotor("front right wheel motor");
    rearLeftMotor = getMotor("rear left wheel motor");
    rearRightMotor = getMotor("rear right wheel motor");
    keyboard = getKeyboard();
    // enabling sensors and actuators
    mLidar->enable(TIME_STEP);
    mLidar->enablePointCloud();
    frontLeftMotor->setPosition(INFINITY);
    frontRightMotor->setPosition(INFINITY);
    rearLeftMotor->setPosition(INFINITY);
    rearRightMotor->setPosition(INFINITY);
    frontLeftMotor->setVelocity(0);
    frontRightMotor->setVelocity(0);
    rearLeftMotor->setVelocity(0);
    rearRightMotor->setVelocity(0);
    // enable/disable keyboard
    std::ifstream keyboardConfig("keyboardConfig.txt", std::ifstream::in);
    if (keyboardConfig.is_open()) {
        std::string line{};
        if (std::getline(keyboardConfig, line)) {
          if (line.find("keyboardControl=true") != std::string::npos) {
            keyboard->enable(TIME_STEP);
            keyboardCheck = 1;
          } else {
            keyboard->disable();
            keyboardCheck = 0;
          }
          keyboardConfig.close();
        }
    }
}

LeaderRobot::~LeaderRobot() {
}

void LeaderRobot::run() {
    while (step(TIME_STEP) != -1) {        
        if (keyboardCheck == 1) {
          keyboardControl();
        }
        // scan environment if coordinate data is empty
        if (OOICoordinates.empty()) {
          scanLidarData(); 
          printLidarData(); 
          assignToScouts(); 
        } // sends it just fine
        auto data = receiveMessage();
        //std::cout << "leader received: " << data.first << " " << data.second << '\n';
        if (!(data.first.empty() && data.second.empty())) {
          std::string colour{};
          colour = (data.second.compare("0") == 0) ? "red" : "green";
          datalog.clear();
          datalog = "\nRobot: " +
                    data.first +
                    " has identified a " +
                    colour +
                    " OOI";
          fileOutput(datalog);
          if (data.second.compare("1") == 0) {
            // if a scout found a green OOI, set the target coordinates
            setTargetPosition(
              OOICoordinates[std::stoi(data.first) - 1].x, 
              OOICoordinates[std::stoi(data.first) - 1].y
            );
          }
        }
        if (!(targetPositionX == 0 && targetPositionY == 0)) {
          // received green OOI coordinate, update output.txt and start moving to target
          datalog = "\nGreen OOI has been found, moving to x:" +
                    std::to_string(OOICoordinates[std::stoi(data.first) - 1].x) +
                    " y:" +
                    std::to_string(OOICoordinates[std::stoi(data.first) - 1].y);
          fileOutput(datalog);
          updateCurrentPosition();
          if (setDirectionToTarget() == true) {
            moveToTarget(1.00);
            if ((frontLeftMotor->getVelocity() == 0) && (rearRightMotor->getVelocity() == 0)) {
              fileOutput("\nSuccessfully arrived at the green OOI"); 
            }
          } 
        }
    };
}

void LeaderRobot::move(double speed) {
    frontLeftMotor->setVelocity(speed);
    frontRightMotor->setVelocity(speed);
    rearLeftMotor->setVelocity(speed);
    rearRightMotor->setVelocity(speed);
}

void LeaderRobot::rotate(double speed) {
    frontLeftMotor->setVelocity(-speed);
    frontRightMotor->setVelocity(speed);
    rearLeftMotor->setVelocity(-speed);
    rearRightMotor->setVelocity(speed);
}

void LeaderRobot::scanLidarData() {
    std::cout << "Scanning for OOI..." << std::endl;
    // retrieves and saves data into data member - this part works
    const WbLidarPoint* pointArray = mLidar->getPointCloud();
    int size = mLidar->getNumberOfPoints();
    for (int i{ 0 }; i < size; i++) {
        PointCoordinates point{};
        point.x = pointArray[i].x;
        point.y = pointArray[i].y;
        pointCloud.push_back(point);
    }
    // loops and stores OOI coordinates
    // std::cout << "extracting pointcloud to ooicoordinates\n";
    bool foundInf = false;
    auto it = pointCloud.begin();
    while (it != pointCloud.end()) {
        if (!std::isinf(it->x) && !std::isinf(it->y)) {
            if (foundInf) {
               OOI ooi{};
               ooi.x = it->x;
               ooi.y = it->y;
               OOICoordinates.push_back(ooi);
               foundInf = false; // reset the flag after storing the point 
            }
        } else {
            foundInf = true; // set the flag if encountered infinite
        }
        ++it; // move the iterator to the next element
    }
    // std::cout << "ooi coordinates\n";
    // for (int i=0; i<static_cast<int>(OOICoordinates.size());i++) {
      // std::cout << OOICoordinates[i].x << "," << OOICoordinates[i].y << "\n";
    // }
    
    std::cout << "Scanning finished.\nPrinting results..." << std::endl;
}

// prints the lidar data to terminal to provide references for calling fileOutput function
void LeaderRobot::printLidarData() {
    datalog.clear();
    if (!OOICoordinates.empty()) {
        for (const auto& coordinate : OOICoordinates) {
            // append one coordinate of OOI into a string and print to terminal and output.txt
            datalog = "\nOOI discovered at x:" +
                      std::to_string(coordinate.x) +
                      " y:" +
                      std::to_string(coordinate.y);
            fileOutput(datalog);
        }
    } else {
        std::cout << "No OOI found in the area!" << std::endl;
    }
}

// assign each OOI coordinates to a robot ID
// bug with printing last datalog twice
void LeaderRobot::assignToScouts() {
    datalog.clear();
    int indexID = 1;
    for (int i = 0; i < static_cast<int>(OOICoordinates.size()); ++i) {
        sendMessage(
            std::to_string(indexID),                // const std::string& toID
            std::to_string(OOICoordinates[i].x), // const std::string& data0
            std::to_string(OOICoordinates[i].y)  // const std::string& data1
        );
        // auto data = receiveMessage();
        // std::cout << "check first data: " << data.first << '\n';
        // std::cout << "check second data: " << data.second << '\n';
        // std::cout << "q length: " << receiver->getQueueLength() << '\n';
        // print updates to terminal and output.txt
        datalog = "\nTarget pose x:" +
                  std::to_string(OOICoordinates[i].x) +
                  " y:" +
                  std::to_string(OOICoordinates[i].y) +
                  " sent to robot " +
                  std::to_string(indexID);
        fileOutput(datalog);
        // ensures indexID cycles through a fixed consecutive value, in this case: 1-3.
        indexID = (indexID % 3) + 1;
    }
}

void LeaderRobot::fileOutput(const std::string& output) {
    std::ofstream outputFile("output.txt", std::ios::app);
    outputFile << output << std::endl;
    std::cout << output << std::endl;
}

// RUN SIMULATION IN MAIN LOOP
int main(int argc, char **argv) {
    std::unique_ptr<LeaderRobot> leader{ std::make_unique<LeaderRobot>() };
    leader->run();
}