#include "M2Spasticity.h"

#define OWNER ((M2Spasticity *)owner)

M2Spasticity::M2Spasticity() {
    robot = new RobotM2();
    STest = new SpasticityTest();

    // Create PRE-DESIGNED State Machine events and state objects.
    calibState = new M2Calib(this, robot);
    standbyState = new M2Transparent(this, robot);
    recordingState = new M2Recording(this, robot);
    experimentState = new M2ArcCircle(this, robot);
    minJerkState = new M2MinJerkPosition(this, robot);
    experimentReturnState = new M2ArcCircleReturn(this, robot);
    testingState = new M2CircleTest(this, robot);

    endCalib = new EndCalib(this);
    goToNextState = new GoToNextState(this);
    goToPrevState = new GoToPrevState(this);
    startRecording = new StartRecording(this);
    endRecording = new EndRecording(this);
    failRecording = new FailRecording(this);
    startTesting = new StartTesting(this);
    endTesting = new EndTesting(this);
    failTesting = new FailTesting(this);
    startTrial = new StartTrial(this);
    startNextVel = new StartNextVel(this);
    startReturn = new StartReturn(this);
    endTrial = new EndTrial(this);
    goToTransparent = new GoToTransparent(this);
    maxForceReturn = new MaxForceReturn(this);

    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
     NewTransition(calibState, endCalib, standbyState);
     NewTransition(standbyState, startRecording, recordingState);
     NewTransition(recordingState, endRecording, minJerkState);
     NewTransition(recordingState, failRecording, standbyState);
     NewTransition(minJerkState, startTesting, testingState);
     NewTransition(testingState, endTesting, minJerkState);
     NewTransition(testingState, failTesting, standbyState);
     NewTransition(minJerkState, startTrial, experimentState);
     NewTransition(minJerkState, startNextVel, experimentState);
     NewTransition(experimentState, startReturn, minJerkState);
     NewTransition(minJerkState, endTrial, standbyState);
     //NewTransition(experimentState, goToNextState, experimentReturnState);
     //NewTransition(experimentReturnState, goToPrevState, experimentState);
     NewTransition(standbyState, maxForceReturn, minJerkState);
     NewTransition(recordingState, goToTransparent, standbyState);
     NewTransition(testingState, goToTransparent, standbyState);
     NewTransition(experimentState, goToTransparent, standbyState);
     NewTransition(minJerkState, goToTransparent, standbyState);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(calibState);
}
M2Spasticity::~M2Spasticity() {
    delete UIserver;
    delete robot;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void M2Spasticity::init() {
    spdlog::debug("M2Spasticity::init()");
    if(robot->initialise()) {
        initialised = true;
        logHelper.initLogger("M2SpasticityLog", "logs/M2Spasticity.csv", LogFormat::CSV, true);
        logHelper.add(time_running, "Time (s)");
        logHelper.add(robot->getEndEffPositionRef(), "Position");
        logHelper.add(robot->getEndEffVelocityRef(), "Velocity");
        logHelper.add(robot->getInteractionForceRef(), "Force");
        logHelper.startLogger();
        UIserver = new FLNLHelper(robot, "127.0.0.1");
        UIserver->registerState(StateIndex); //example to register a continuous value
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

void M2Spasticity::end() {
    if(initialised) {
        if(logHelper.isStarted())
            logHelper.endLog();
        UIserver->closeConnection();
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
void M2Spasticity::hwStateUpdate(void) {
    time_running = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time_init).count()) / 1e6;
    robot->updateRobot();
    UIserver->sendState();
}



bool M2Spasticity::EndCalib::check() {
    return OWNER->calibState->isCalibDone();
}


bool M2Spasticity::GoToNextState::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==1) )
        return true;

    //Check incoming command requesting state change
    if ( OWNER->UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        OWNER->UIserver->getCmd(cmd, v);
        if (cmd == "GTNS") { //Go To Next State command received
            //Acknowledge
            OWNER->UIserver->clearCmd();
            OWNER->UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool M2Spasticity::GoToPrevState::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==2) )
        return true;

    //Check incoming command requesting state change
    if ( OWNER->UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        OWNER->UIserver->getCmd(cmd, v);
        if (cmd == "GTPS") { //Go To Previous State command received
            //Acknowledge
            OWNER->UIserver->clearCmd();
            OWNER->UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool M2Spasticity::StartRecording::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==3) )
        return true;

    //Check incoming command requesting state change
    if ( OWNER->UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        OWNER->UIserver->getCmd(cmd, v);
        if (cmd == "RECD") { //Start Recording command received
            //Acknowledge
            OWNER->UIserver->clearCmd();
            OWNER->UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool M2Spasticity::EndRecording::check() {
    return OWNER->recordingState->isRecordingDone();
}


bool M2Spasticity::FailRecording::check() {
    return OWNER->recordingState->isRecordingError();
}


bool M2Spasticity::StartTesting::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==4) )
        return true;

    //Check incoming command requesting state change
    if ( OWNER->UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        OWNER->UIserver->getCmd(cmd, v);
        if (cmd == "TEST") { //Start Testing command received
            //Acknowledge
            OWNER->UIserver->clearCmd();
            OWNER->UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool M2Spasticity::EndTesting::check() {
    return OWNER->testingState->isTestingDone();
}


bool M2Spasticity::FailTesting::check() {
    return OWNER->testingState->isTestingError();
}


bool M2Spasticity::StartTrial::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==5) )
        return true;

    //Check incoming command requesting state change
    if ( OWNER->UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        OWNER->UIserver->getCmd(cmd, v);
        if (cmd == "TRIA") { //Start Trial command received
            //Acknowledge
            OWNER->UIserver->clearCmd();
            OWNER->UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool M2Spasticity::StartNextVel::check() {
    return OWNER->minJerkState->GoToNextVel();
}


bool M2Spasticity::StartReturn::check() {
    return OWNER->experimentState->GoToStartPt();
}


bool M2Spasticity::EndTrial::check() {
    return OWNER->minJerkState->isTrialDone();
}


bool M2Spasticity::MaxForceReturn::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==8) )
        return true;

    //Check incoming command requesting state change
    if ( OWNER->UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        OWNER->UIserver->getCmd(cmd, v);
        if (cmd == "MFRT") { //Max Force Return command received
            //Acknowledge
            OWNER->UIserver->clearCmd();
            OWNER->UIserver->sendCmd(string("OK"));
            return true;
        }
    }

    //Otherwise false
    return false;
}


bool M2Spasticity::GoToTransparent::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==9))
        return true;

    if (OWNER->goToTransparentFlag)
    {
        OWNER->goToTransparentFlag = false;
        return true;
    }

    //Check incoming command requesting state change
    if ( OWNER->UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        OWNER->UIserver->getCmd(cmd, v);
        if (cmd == "REST") { //Go To Transparent command received
            //Acknowledge
            OWNER->UIserver->clearCmd();
            OWNER->UIserver->sendCmd(string("OK"));
            OWNER->StateIndex = 9.;
            return true;
        }
    }

    //Otherwise false
    return false;
}



bool M2Spasticity::configureMasterPDOs() {
    spdlog::debug("M2Spasticity::configureMasterPDOs()");
    return robot->configureMasterPDOs();
}
