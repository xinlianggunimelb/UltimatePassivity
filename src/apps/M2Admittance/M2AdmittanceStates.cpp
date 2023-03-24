#include "M2AdmittanceStates.h"
#include "M2Admittance.h"
#include <stdlib.h>
#include <bits/stdc++.h>
//#include <complex>

#define OWNER ((M2Admittance *)owner)


int EveryN=1;//Process every N samples: reduce frequency

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

VM2 myVE(VM2 X, VM2 dX, VM2 Fm, Eigen::Matrix2d B, Eigen::Matrix2d M, double dt)
{
    Eigen::Matrix2d realM;
    Eigen::Matrix2d realB;

    if(X(0)<0.20)
    {
     realB(0,0) = 15.0;
     realB(1,1) = 15.0;
     realM(0,0) = 10.0;
     realM(1,1) = 10.0;
    }
    else
    {
     realB = B;
     realM = M;
    }

    Eigen::Matrix2d Operator;
    Operator(0,0) = 1/(realM(0,0) + realB(0,0)*dt);
    Operator(1,1) = 1/(realM(1,1) + realB(1,1)*dt);

    return Operator*(Fm*dt + realM*dX);
}

VM2 PsvObsv(VM2 E_obs_ls, VM2 Fm, VM2 Fm_ls, VM2 V_ve, Eigen::Matrix2d C_diss_ls, double dt)
{
    VM2 E_obs;
    E_obs = E_obs_ls + V_ve.cwiseProduct(Fm)*dt + C_diss_ls*Fm_ls.cwiseProduct(Fm_ls)*dt;
    return E_obs;
}

VM2 PsvObsvSimple(VM2 E_obs_ls, VM2 Fm, VM2 dX, double dt)
{
    VM2 E_obs;
    E_obs = E_obs_ls + dX.cwiseProduct(Fm)*dt;
    return E_obs;
}

VM2 myPOobs(VM2 Vm, VM2 Fm, double dt)
{
    VM2 Em;
    Em = Vm.cwiseProduct(Fm);
    return Em;
}

VM2 myPOerror(VM2 Vm, VM2 Vd, VM2 Fm, double dt)
{
   VM2 Pref;
   VM2 Pobs;
   VM2 Pdev;

   Pobs = Vm.cwiseProduct(Fm);
   Pref = Vd.cwiseProduct(Fm);
   Pdev = Pref - Pobs;
   return Pdev;
}

VM2 impedance(Eigen::Matrix2d K, Eigen::Matrix2d D, VM2 X0, VM2 X, VM2 dX, VM2 dXd=VM2::Zero()) {
    return K*(X0-X) + D*(dXd-dX); //K is equivlent to the P gain in position control, D is equivlent to the D gain in position control the reference point is the messured velocity
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

    startTime=elapsedTime;
    Xi = robot->getEndEffPosition();
    VM2 destination;
    destination(0)=0.3;
    destination(1)=0.2;
    Xf = destination;
    T=3; //Trajectory Time
    k_i=1.;
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
        if(iterations%10==1) {
        robot->printStatus();
        }
}

void M2MinJerkPosition::exitCode(void) {
    robot->setJointVelocity(VM2::Zero());
}


// M2Admittance1: Classical POPC
void M2Admittance1::entryCode(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    //Virtual Environment
    M(0,0) = M(1,1) = 0.2;
    B(0,0) = B(1,1) = 0.1;
    //B1(0,0) = B1(1,1) = 3.0;
    //Damper for dissipation
    C_diss(0,0) = C_diss(1,1) = 0.0;
    C_diss_ls(0,0) = C_diss_ls(1,1) = 0.0;

    Obsv_T = 10000000; //an arbitrary large value
    i = 0;

    E_class(0) = E_class(1) = 2.0;
    E_diss(0) = E_diss(1) = 0.0;
    //alpha(0) = alpha(1) = 20000;

    //E_obs(0) = E_obs(1) = 0.0;
    E_obs(0) = E_obs(1) = E_class(0) + 1.0;
    E_obs_ls(0) = E_obs_ls(1) = E_class(0) +1.0;

    X(VM2::Zero());
    dX(VM2::Zero());
    Fm(VM2::Zero());
    Fm_ls(VM2::Zero());
    Vd(VM2::Zero());
    //V_error(VM2::Zero());
    //P_error(VM2::Zero());
    V_ve(VM2::Zero());
    V_diss(VM2::Zero());

    stateLogger.initLogger("M2Admittance1State", "logs/M2Admittance1State.csv", LogFormat::CSV, true);
    stateLogger.add(elapsedTime, "%Time (s)");
    stateLogger.add(robot->getEndEffPositionRef(), "Position");
    stateLogger.add(robot->getEndEffVelocityRef(), "Velocity");
    stateLogger.add(robot->getInteractionForceRef(), "Force");
    stateLogger.add(B(0,0), "Virtual_damping_X");
    //stateLogger.add(B(1,1), "Virtual_damping_Y");
    stateLogger.add(M(0,0), "Virtual_mass_X");
    stateLogger.add(E_class, "Energy_threshold");
    stateLogger.add(E_obs, "Observed_Energy");
    stateLogger.add(E_diss, "Dissipation_Energy");
    //stateLogger.add(V_error, "Velocity_error");
    //stateLogger.add(P_error, "Power_PO");
    stateLogger.add(V_ve(0), "VE_velocity_X");
    stateLogger.add(V_diss(0), "Diss_velocity_X");
    stateLogger.add(Vd(0), "Desired_velocity_X");
    stateLogger.add(C_diss(0,0), "1/b_X");
    stateLogger.startLogger();
}

void M2Admittance1::duringCode(void) {
    //VM2 X, dX, Fm, Vd;
    //get robot position and velocity and force mesaure
    X = robot->getEndEffPosition();
    dX = robot->getEndEffVelocity();
    Fm = robot->getInteractionForceRef();

    //Change Virtual Damping
    if(robot->keyboard->getQ()) {
        B(0,0)+=0.01;
        B(1,1)+=0.01;
        std::cout << B <<std::endl;
    }
    if(robot->keyboard->getA()) {
        B(0,0)-=0.01;
        B(1,1)-=0.01;
        std::cout << B <<std::endl;
    }
    //Change Virtual Mass
    if(robot->keyboard->getW()) {
        M(0,0)+=0.01;
        M(1,1)+=0.01;
        std::cout << M <<std::endl;
    }
    if(robot->keyboard->getS()) {
        M(0,0)-=0.01;
        M(1,1)-=0.01;
        std::cout << M <<std::endl;
    }

    V_ve = myVE(X, dX, Fm, B, M, dt);

    //V_error = Vd - dX;
    //P_error = myPOerror(dX,Vd,Fm,dt)*dt;

    // Passivity Observer
    if (i < Obsv_T) {
        //E_obs = E_obs + myPOobs(dX, Fm, dt)*dt;
        E_obs = PsvObsv(E_obs_ls, Fm, Fm_ls, V_ve, C_diss_ls, dt);
        //E_obs = PsvObsvSimple(E_obs_ls, Fm, dX, dt);
        i += 1;
    }
    if (i >= Obsv_T) {
        i = 0;
        //E_obs(VM2::Zero());
        E_obs(0) = E_obs(1) = 0;
        std::cout << "PO is reseted" << std::endl;
    }

    //E_class triggered
    /*
    if (E_obs(0) <= E_class(0)) {
        B(0,0) = B1(0,0) + alpha(0)*abs(P_error(0))*sign(P_error(0));
        B(0,0) = max(B(0,0), B1(0,0));
    }
    else B(0,0) = B1(0,0);
    if (E_obs(1) <= E_class(1)) {
        B(1,1) = B1(1,1) + alpha(1)*abs(P_error(1))*sign(P_error(1));
        B(1,1) = max(B(1,1), B1(1,1));
    }
    else B(1,1) = B1(1,1);
    B(0,0) = max(B(0,0), 0.01);
    B(1,1) = max(B(1,1), 0.01);
    */
    if (E_obs(0) < E_class(0)) {
        E_diss(0) = E_class(0) - E_obs(0); //energy to be dissipated
        C_diss(0,0) = E_diss(0) / (Fm(0)*Fm(0)*dt);
    }
    else {
        E_diss(0) = 0.0;
        C_diss(0,0) = 0.0;
    }
    if (E_obs(1) < E_class(1)) {
        E_diss(1) = E_class(1) - E_obs(1); //energy to be dissipated
        C_diss(1,1) = E_diss(1) / (Fm(1)*Fm(1)*dt);
    }
    else {
        E_diss(1) = 0.0;
        C_diss(1,1) = 0.0;
    }

    //Vd = myVE(X, dX, Fm, B, M, dt);
    V_diss(0) = C_diss(0,0)*Fm(0);
    V_diss(1) = C_diss(1,1)*Fm(1);
    Vd = V_ve + V_diss;
    Vd(1) = 0.0; //lock y axis

    E_obs_ls = E_obs;
    Fm_ls = Fm;
    C_diss_ls = C_diss;

    stateLogger.recordLogData();
    if(iterations%10==1) {
        //robot->printStatus();
        //std::cout << "i=[ " << i << " ]\t" ;
        std::cout << "Mass_X=[ " << M(0,0) << " ]\t" ;
        std::cout << "Damping_X=[ " << B(0,0) << " ]\t" ;
        //std::cout << "Damping_Y=[ " << B(1,1) << " ]\t" <<std::endl;
        //std::cout << "E_class=[ " << E_class.transpose() << " ]\t" ;
        std::cout << "Energy=[ " << E_obs.transpose() << " ]\t" ;
        std::cout << "E_diss=[ " << E_diss.transpose() << " ]\t" ;
        std::cout << "V_ve=[ " << V_ve.transpose() << " ]\t" ;
        std::cout << "V_diss=[ " << V_diss.transpose() << " ]\t" ;
        std::cout << "C_X=[ " << C_diss(0,0) << " ]\t" <<std::endl;
    }

    //apply velocity
    robot->setEndEffVelocity(Vd);
}

void M2Admittance1::exitCode(void) {
    robot->setJointVelocity(VM2::Zero());
}


// 2Admittance2: Ultimate Passivity
void M2Admittance2::entryCode(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());

    M(0,0) = M(1,1) = 0.20;
    B(0,0) = B(1,1) = 0.08;

    //B1(0,0) = B1(1,1) = 1.2;
    B1(0,0) = B1(1,1) = 0.0;
    B2(0,0) = B2(1,1) = 50.0;
    //B(0,0)=B1(0,0);
    //B(1,1)=B1(1,1);

    //Damper for dissipation
    C_diss(0,0) = C_diss(1,1) = 0.0;
    C_diss_ls(0,0) = C_diss_ls(1,1) = 0.0;

    E_obs(0) = E_obs(1) = 0.0;
    E_obs_ls(0) = E_obs_ls(1) = 0.0;

    Obsv_T = 10000000;
    i = 0;

    X(VM2::Zero());
    dX(VM2::Zero());
    Fm(VM2::Zero());
    Fm_ls(VM2::Zero());
    Vd(VM2::Zero());
    //V_error(VM2::Zero());
    V_ve(VM2::Zero());
    V_diss(VM2::Zero());

    E_lower(0) = E_lower(1) = -0.5;
    E_upper(0) = E_upper(1) = 2.0;

    stateLogger.initLogger("M2Admittance2State", "logs/M2Admittance2State.csv", LogFormat::CSV, true);
    stateLogger.add(elapsedTime, "%Time (s)");
    stateLogger.add(robot->getEndEffPositionRef(), "Position");
    stateLogger.add(robot->getEndEffVelocityRef(), "Velocity");
    stateLogger.add(robot->getInteractionForceRef(), "Force");
    stateLogger.add(B(0,0), "Virtual_damping_X");
    //stateLogger.add(B(1,1), "Virtual_damping_Y");
    stateLogger.add(M(0,0), "Virtual_mass_X");
    stateLogger.add(E_upper, "Energy_threshold_upper");
    stateLogger.add(E_lower, "Energy_threshold_lower");
    stateLogger.add(E_obs, "Observed_Energy");
    stateLogger.add(V_ve(0), "VE_velocity_X");
    stateLogger.add(V_diss(0), "Diss_velocity_X");
    stateLogger.add(Vd(0), "Desired_velocity_X");
    stateLogger.add(C_diss(0,0), "1/b_X");
    stateLogger.startLogger();
}

void M2Admittance2::duringCode(void) {
    //get robot position and velocity and force mesaure
    X = robot->getEndEffPosition();
    dX = robot->getEndEffVelocity();
    Fm = robot->getInteractionForceRef();

    //Change E_UltPsv upper threshold
    if(robot->keyboard->getQ()) {
        E_upper(0)+=0.1;
        E_upper(1)+=0.1;
        std::cout << E_upper <<std::endl;
    }
    if(robot->keyboard->getA()) {
        E_upper(0)-=0.1;
        E_upper(1)-=0.1;
        std::cout << E_upper <<std::endl;
    }
    //Change E_UltPsv lower threshold
    if(robot->keyboard->getW()) {
        E_lower(0)+=0.1;
        E_lower(1)+=0.1;
        std::cout << E_lower <<std::endl;
    }
    if(robot->keyboard->getS()) {
        E_lower(0)-=0.1;
        E_lower(1)-=0.1;
        std::cout << E_lower <<std::endl;
    }

    V_ve = myVE(X, dX, Fm, B, M, dt);

    // Passivity Observer
    if (i < Obsv_T) {
        //E_obs = E_obs + myPOobs(dX, Fm, dt)*dt;
        //E_obs = PsvObsv(E_obs_ls, Fm, Fm_ls, V_ve, C_diss_ls, dt);
        E_obs = PsvObsvSimple(E_obs_ls, Fm, dX, dt);
        i += 1;
    }
    if (i >= Obsv_T) {
        i = 0;
        //E_obs(VM2::Zero());
        E_obs(0) = E_obs(1) = 0;
        std::cout << "PO is reseted" << std::endl;
    }

    //Switching Law
    /*
    if (E_obs(0) <= E_lower(0)) {
        B(0,0) = B2(0,0);
    }
    if (E_obs(0) >= E_upper(0)) {
        B(0,0) = B1(0,0);
    }
    else {
        B(0,0) = B(0,0);
    }

    if (E_obs(1) <= E_lower(1)) {
        B(1,1) = B2(1,1);
    }
    if (E_obs(1) >= E_upper(1)) {
        B(1,1) = B1(1,1);
    }
    else {
        B(1,1) = B(1,1);
    }
    B(0,0) = max(B(0,0), 0.01);
    B(1,1) = max(B(1,1), 0.01);
    */
    if (E_obs(0) <= E_lower(0)) {
        C_diss(0,0) = 1.0 / B2(0,0);
    }
    if (E_obs(0) >= E_upper(0)) {
        C_diss(0,0) = 0.0;
    }
    else {
        C_diss(0,0) = C_diss(0,0);
    }

    if (E_obs(1) <= E_lower(1)) {
        C_diss(1,1) = 1.0 / B2(1,1);
    }
    if (E_obs(1) >= E_upper(1)) {
        C_diss(1,1) = 0.0;
    }
    else {
        C_diss(1,1) = C_diss(1,1);
    }

    //Vd = myVE(X, dX, Fm, B, M, dt);
    V_diss(0) = C_diss(0,0)*Fm(0);
    V_diss(1) = C_diss(1,1)*Fm(1);
    Vd = V_ve + V_diss;
    Vd(1) = 0.0; //lock y axis
    //apply velocity
    robot->setEndEffVelocity(Vd);

    E_obs_ls = E_obs;
    Fm_ls = Fm;
    C_diss_ls = C_diss;

    stateLogger.recordLogData();
    if(iterations%10==1) {
        //robot->printStatus();
        std::cout << "Upper_E_X=[ " << E_upper(0) << " ]\t" ;
        std::cout << "Lower_E_X=[ " << E_lower(0) << " ]\t" ;
        std::cout << "Energy=[ " << E_obs.transpose() << " ]\t" ;
        //std::cout << "Damping_x=[ " << B(0,0) << " ]\t" ;
        //std::cout << "Damping_y=[ " << B(1,1) << " ]\t" <<std::endl;
        std::cout << "V_ve=[ " << V_ve.transpose() << " ]\t" ;
        std::cout << "V_diss=[ " << V_diss.transpose() << " ]\t" ;
        std::cout << "Damping_X=[ " << C_diss(0,0) << " ]\t" <<std::endl;
    }
}

void M2Admittance2::exitCode(void) {
    robot->setJointVelocity(VM2::Zero());
}

