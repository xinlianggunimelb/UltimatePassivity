#include "M2Admittance.h"

#define OWNER ((M2Admittance *)owner)

M2Admittance::M2Admittance() {
    robot = new RobotM2();
    STest = new AdmittanceTest();

    // Create PRE-DESIGNED State Machine events and state objects.
    calibState = new M2Calib(this, robot); // Calibration which state ?
    standbyState = new M2Transparent(this, robot); // Transparent state ?
    goToP0State = new M2MinJerkPosition(this, robot); //
    admittance1State = new M2Admittance1(this, robot);
    admittance2State = new M2Admittance2(this, robot);
    //admittance3State = new M2Admittance3(this, robot);

    endCalib = new EndCalib(this);
    goToOne = new GoToOne(this);
    goToTwo = new GoToTwo(this);
    //goToThree = new GoToThree(this);
    goToNextState = new GoToNextState(this);
    goToPrevState = new GoToPrevState(this);
    goToTransparent = new GoToTransparent(this);

    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
     NewTransition(calibState, endCalib, goToP0State);
     NewTransition(goToP0State, goToOne, admittance1State);
     NewTransition(goToP0State, goToTwo, admittance2State);
     //NewTransition(goToP0State, goToThree, admittance3State);
     NewTransition(admittance1State, goToNextState, goToP0State);
     NewTransition(admittance2State, goToNextState, goToP0State);
     //NewTransition(admittance3State, goToNextState, goToP0State);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(calibState);
}
M2Admittance::~M2Admittance() {
    delete robot;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void M2Admittance::init() {
    spdlog::debug("M2Admittance::init()");
    if(robot->initialise()) {
        initialised = true;
        logHelper.initLogger("M2AdmittanceLog", "logs/M2Admittance.csv", LogFormat::CSV, true);
        logHelper.add(time_running, "Time (s)");
        logHelper.add(robot->getEndEffPositionRef(), "Position");
        logHelper.add(robot->getEndEffVelocityRef(), "Velocity");
        logHelper.add(robot->getInteractionForceRef(), "Force");
        logHelper.startLogger();
    }
    else {
        initialised = false;
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
    running = true;
    time_init = std::chrono::steady_clock::now();
    time_running = 0;
}

void M2Admittance::end() {
    if(initialised) {
        if(logHelper.isStarted())
            logHelper.endLog();
        currentState->exit();
        robot->disable();
    }
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void M2Admittance::hwStateUpdate(void) {
    time_running = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time_init).count()) / 1e6;
    robot->updateRobot();
}

bool M2Admittance::EndCalib::check() {
    return OWNER->calibState->isCalibDone();
}

bool M2Admittance::GoToOne::check() {
    //keyboard or joystick press
    if ( OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==1 )
        return true;

    //Otherwise false
    return false;
}
bool M2Admittance::GoToTwo::check() {
    //keyboard or joystick press
    if ( OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==2 )
        return true;

    //Otherwise false
    return false;
}
/*
bool M2Admittance::GoToThree::check() {
    //keyboard or joystick press
    if ( OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==3 )
        return true;

    //Otherwise false
    return false;
}
*/

bool M2Admittance::GoToNextState::check() {
    //keyboard or joystick press
    if ( OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==9 )
        return true;

    //Otherwise false
    return false;
}

bool M2Admittance::GoToPrevState::check() {
    //keyboard or joystick press
    if ( OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==8 )
        return true;

    //Otherwise false
    return false;
}

bool M2Admittance::GoToTransparent::check() {
    //keyboard or joystick press
    if ( OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==7 )
        return true;

    if (OWNER->goToTransparentFlag)
    {
        OWNER->goToTransparentFlag = false;
        return true;
    }

    //Otherwise false
    return false;
}

bool M2Admittance::configureMasterPDOs() {
    spdlog::debug("M2Admittance::configureMasterPDOs()");
    return robot->configureMasterPDOs();
}
