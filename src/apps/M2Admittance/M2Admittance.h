/**
 * \file M2Admittance.h
 * \author Vincent Crocher
 * /brief The <code>M2Admittance</code> class represents an example implementation of an M2 state machine.
 * \version 0.1
 * \date 2020-12-09
 *
 * \copyright Copyright (c) 2020
 *
 */
#ifndef M2_SM_H
#define M2_SM_H

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <csignal> //For raise()

#include "RobotM2.h"
#include "StateMachine.h"
#include "FLNLHelper.h"

// State Classes
#include "M2AdmittanceStates.h"


//Admittance test variables
struct AdmittanceTest
{
};

/**
 * @brief Example implementation of a StateMachine for the M2Robot class. States should implemented M2DemoState
 *
 */
class M2Admittance : public StateMachine {
   public:
    bool running = false;
    std::chrono::steady_clock::time_point time_init; // initial time that machine started
    double time_running; // time passed after initialisation in [s]
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M2Admittance();
    ~M2Admittance();
    void init();
    void end();

    void hwStateUpdate();
    bool configureMasterPDOs();

    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    M2Calib *calibState;
    M2Transparent *standbyState;
    M2Admittance1 *admittance1State;
    M2Admittance2 *admittance2State;
    M2Admittance3 *admittance3State;
    M2MinJerkPosition* goToP0State;

    AdmittanceTest *STest;

    bool goToTransparentFlag = false;


   protected:
    RobotM2 *robot;         /*!< Pointer to the Robot*/

   private:
    EventObject(EndCalib) * endCalib;
    EventObject(GoToOne) * goToOne;
    EventObject(GoToTwo) * goToTwo;
    EventObject(GoToThree) * goToThree;
    EventObject(GoToNextState) * goToNextState;
    EventObject(GoToPrevState) * goToPrevState;
    EventObject(GoToTransparent) * goToTransparent;

};

#endif /*M2_SM_H*/
