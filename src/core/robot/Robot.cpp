/**
 * @file Robot.cpp
 * @author Justin Fong
 * @brief Generic Abstract Robot class, which includes joints and a trajectory generator, to be used
 *          with a CAN-based robot device
 * @version 0.1
 * @date 2020-04-17
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "Robot.h"

#include "DebugMacro.h"

Robot::Robot(){
    DEBUG_OUT("Robot object created")
}

Robot::~Robot() {
    DEBUG_OUT("Robot object deleted")
}

bool Robot::initialise() {
    if (initialiseJoints()) {
        if (initialiseNetwork()) {
            if (initialiseInputs()) {
                return true;
            } else {
                return false;
            }
        }
    } else
        return false;
}


bool Robot::stop() {
    return true;
}

void Robot::updateRobot() {
//    std::cout << "Robot::updateRobot" << std::endl;
    for (auto joint : joints)
        joint->updateValue();
    for (auto input : inputs)
        input->updateInput();
}

void Robot::printStatus() {
    std::cout << "Robot Joint Angles";
    for (auto joint : joints)
        std::cout << joint->getPosition() << " ";
    std::cout << std::endl;
}

void Robot::printJointStatus(int J_i) {
    joints[J_i]->getStatus();
}
