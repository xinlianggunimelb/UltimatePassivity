#include "M2AdmittanceStates.h"
#include "M2Admittance.h"
#include <stdlib.h>
#include <bits/stdc++.h>
//#include <complex>

#define OWNER ((M2Admittance *)owner)


double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

VM2 impedance(Eigen::Matrix2d K, Eigen::Matrix2d D, VM2 X0, VM2 X, VM2 dX, VM2 dXd=VM2::Zero()) {
    return K*(X0-X) + D*(dXd-dX);
}

//minJerk(X0, Xf, T, t, &X, &dX)
double JerkIt(VM2 X0, VM2 Xf, double T, double t, VM2 &Xd, VM2 &dXd) {
    t = std::max(std::min(t, T), .0); //Bound time
    double tn=std::max(std::min(t/T, 1.0), .0);//Normalised time bounded 0-1
    double tn3=pow(tn,3.);
    double tn4=tn*tn3;
    double tn5=tn*tn4;
    Xd = X0 + ( (X0-Xf) * (15.*tn4-6.*tn5-10.*tn3) );
    dXd = (X0-Xf) * (4.*15.*tn4-5.*6.*tn5-10.*3*tn3)/t;
    return tn;
}


void M2Calib::entryCode(void) {
    calibDone=false;
    for(unsigned int i=0; i<2; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    robot->decalibrate();
    robot->initTorqueControl();
   // robot -> printStatus();
    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}
//Move slowly on each joint until max force detected
void M2Calib::duringCode(void) {
    VM2 tau(0, 0);

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    VM2 vel=robot->getVelocity();
    double b = 3;
    for(int i=0; i<vel.size(); i++) {
        tau(i) = -std::min(std::max(20 - b * vel(i), .0), 20.);
        if(stop_reached_time(i)>1) {
            at_stop[i]=true;
        }
        if(abs(vel(i))<0.005) {
            stop_reached_time(i) += dt;
        }
    }

    //Switch to gravity control when done
    if(robot->isCalibrated()) {
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
        calibDone=true; //Trigger event
    }
    else {
        //If all joints are calibrated
        if(at_stop[0] && at_stop[1]) {
            robot->applyCalibration();
            std::cout << "OK." << std::endl;
        }
        else {
            robot->setJointTorque(tau);
            if(iterations%100==1) {
                std::cout << "." << std::flush;
            }
        }
    }
}
void M2Calib::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}


void M2Transparent::entryCode(void) {
    robot->initTorqueControl();
    ForceP(0,0) = 1.3;
    ForceP(1,1) = 1.4;
}
void M2Transparent::duringCode(void) {

    //Smooth transition in case a mass is set at startup
    //double settling_time = 3.0;
    //double t=elapsedTime>settling_time?1.0:elapsedTime/settling_time;

    //Apply corresponding force
    VM2 f_m = robot->getInteractionForceRef();
    robot->setEndEffForce(ForceP*f_m);

    if(iterations%100==1) {
        robot->printStatus();
    }

    //Read keyboard inputs to change gain
    if(robot->keyboard->getS()) {
        ForceP(1,1)-=0.1;
        std::cout << ForceP <<std::endl;
        std::cout << ForceP*f_m <<std::endl;
    }
    if(robot->keyboard->getW()) {
        ForceP(1,1)+=0.1;
        std::cout << ForceP(1,1) <<std::endl;
        std::cout << ForceP*f_m <<std::endl;
    }
}
void M2Transparent::exitCode(void) {
    robot->setEndEffForce(VM2::Zero());
}

void M2MinJerkPosition::entryCode(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    goToNextVel=false;
    trialDone=false;

    /*startTime=elapsedTime;
    Xi = robot->getEndEffPosition();
    Xf = OWNER->STest->global_start_point;
    T=5; //Trajectory Time
    k_i=1.;*/
}
void M2MinJerkPosition::duringCode(void) {
    VM2 Xd, dXd;
    //Compute current desired interpolated point
    double status=JerkIt(Xi, Xf, T, elapsedTime-startTime, Xd, dXd);
    //Apply position control
    if(robot->isEnabled())
    {
        robot->setEndEffVelocity(dXd+k_i*(Xd-robot->getEndEffPosition()));
    }
    else
    {
        OWNER->goToTransparentFlag = true;
    }
/*
    //distance to the starting point
    double threshold = 0.01;
    VM2 distanceStPt=OWNER->STest->global_start_point-robot->getEndEffPosition();

    //Have we reached a point?
    if (status>=1. && iterations%100==1){
       //check if we reach the starting point
       if(abs(distanceStPt[0])<=threshold && abs(distanceStPt[1])<=threshold){
            //std::cout << "OK. \n";
            if (OWNER->STest->movement_loop==0 && OWNER->StateIndex==3.){
                OWNER->StateIndex=4.;
            }
            if (OWNER->STest->movement_loop==0 && OWNER->StateIndex==7.){
                OWNER->StateIndex=8.;
            }
            if (OWNER->STest->movement_loop>=1 && OWNER->STest->movement_loop<=8){
                goToNextVel=true; //Trigger event: go to next velocity in one trial
            }
            if (OWNER->STest->movement_loop>=9){
                OWNER->StateIndex = 20.;
                OWNER->STest->movement_loop=0;
                trialDone=true;
            }
       }
       else{
           OWNER->StateIndex = 23.;
           OWNER->goToTransparentFlag = true;
       }
    }*/
}
void M2MinJerkPosition::exitCode(void) {
    // std::cout << "Ready... \n";
    robot->setJointVelocity(VM2::Zero());
}
