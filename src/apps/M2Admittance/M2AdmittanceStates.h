/**
 * /file M2AdmittanceStates.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2020-12-09
 *
 * \copyright Copyright (c) 2020
 *
 */

#ifndef M2DemoSTATE_H_DEF
#define M2DemoSTATE_H_DEF

#include <time.h>
#include <iostream>

#include "RobotM2.h"
#include "State.h"

using namespace std;


/**
 * \brief Conversion from a timespec structure to seconds (double)
 *
 */
double timeval_to_sec(struct timespec *ts);

/**
 * \brief Generic state type for used with M2Admittance, providing running time and iterations number.
 *
 */
class M2TimedState : public State {
   protected:
    /**
    *  \todo Might be good to make these Const
    *
    */
    RobotM2 *robot;                               /*<!Pointer to state machines robot object*/

    M2TimedState(StateMachine *m, RobotM2 *M2, const char *name = NULL): State(m, name), robot(M2){};
   private:
    void entry(void) final {
        std::cout
        << "==================================" << std::endl
        << " STARTING  " << getName() << std::endl
        << "----------------------------------" << std::endl
        << std::endl;

        //Timing
        clock_gettime(CLOCK_MONOTONIC, &initTime);
        lastTime = timeval_to_sec(&initTime);

        elapsedTime=0;
        iterations=0;

        //Actual state entry
        entryCode();
    };
    void during(void) final {
        //Compute some basic time values
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);

        double now = timeval_to_sec(&ts);
        elapsedTime = (now-timeval_to_sec(&initTime));
        dt = now - lastTime;
        lastTime = now;

        iterations++;

        //Actual state during
        duringCode();
    };
    void exit(void) final {
        exitCode();
        std::cout
        << "----------------------------------" << std::endl
        << "EXIT "<< getName() << std::endl
        << "==================================" << std::endl
        << std::endl;
    };

   public:
    virtual void entryCode(){};
    virtual void duringCode(){};
    virtual void exitCode(){};


   protected:
    struct timespec initTime;   /*<! Time of state init */
    double lastTime;            /*<! Time of last during() call (in seconds since state init())*/
    double elapsedTime;         /*<! Time since state init() in seconds*/
    double dt;                  /*<! Time between last two during() calls (in seconds)*/
    unsigned long int iterations;
};


class M2DemoState : public M2TimedState {

   public:
    M2DemoState(StateMachine *m, RobotM2 *M2, const char *name = "M2 Test State"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    VM2 qi, Xi, tau;
};


/**
 * \brief Position calibration of M2. Go to the bottom left stops of robot at constant torque for absolute position calibration.
 *
 */
class M2Calib : public M2TimedState {

   public:
    M2Calib(StateMachine *m, RobotM2 *M2, const char *name = "M2 Calib State"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isCalibDone() {return calibDone;}

   private:
     VM2 stop_reached_time;
     bool at_stop[2];
     bool calibDone=false;
};


/**
 * \brief Provide end-effector mass compensation on M2. Mass is controllable through keyboard inputs.
 *
 */
class M2Transparent : public M2TimedState {

   public:
    M2Transparent(StateMachine *m, RobotM2 *M2, const char *name = "M2 Transparent"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    Eigen::Matrix2d ForceP;
};


/**
 * \brief Point to tpoint position control with min jerk trajectory interpolation
 *
 */
class M2MinJerkPosition: public M2TimedState {

   public:
    M2MinJerkPosition(StateMachine *m, RobotM2 *M2, const char *name = "M2 Demo Minimum Jerk Position"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool GoToNextVel() {return goToNextVel;}
    bool isTrialDone() {return trialDone;}

   private:
    bool goToNextVel=false;
    bool trialDone=false;
    double startTime;
    VM2 Xi, Xf;
    double T;
    float k_i=1.; //Integral gain
};


/**
 * \brief Admittance Lawrence algo 1
 *
 */
class M2Admittance1: public M2TimedState {

   public:
    M2Admittance1(StateMachine *m, RobotM2 *M2, const char *name = "M2 Admittance 1"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    LogHelper stateLogger;

    VM2 E_obs;
    Eigen::Matrix2d B;
    Eigen::Matrix2d M;
    Eigen::Matrix2d Operator;
    Eigen::Matrix2d B1, B2;

    VM2 X, dX;
    VM2 V_error, P_error;
    VM2 Fm;
    VM2 Vd;
    VM2 E_class;
    VM2 alpha;
    int Obsv_T;
    int i;
};


/**
 * \brief Admittance Lawrence algo 2
 *
 */
class M2Admittance2: public M2TimedState {

   public:
    M2Admittance2(StateMachine *m, RobotM2 *M2, const char *name = "M2 Admittance 2"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    LogHelper stateLogger;

    VM2 E_obs;
    Eigen::Matrix2d B, Bd;
    Eigen::Matrix2d M, Md;
    Eigen::Matrix2d Operator;
    Eigen::Matrix2d B1, B2;

    VM2 X, dX;
    VM2 V_error;
    VM2 Fm;
    VM2 Vd;
    VM2 E_upper, E_lower;
    int Obsv_T, i;
};


#endif
