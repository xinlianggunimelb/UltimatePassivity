#include "M2SpasticityStates.h"
#include "M2Spasticity.h"
#include <stdlib.h>
#include <bits/stdc++.h>
//#include <complex>

#define OWNER ((M2Spasticity *)owner)


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
    for(unsigned int i=0; i<vel.size(); i++) {
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
    /*if(robot->keyboard->getS()) {
        P(1,1)-=0.1;
        std::cout << P <<std::endl;
        std::cout << P*f_m <<std::endl;
    }
    if(robot->keyboard->getW()) {
        P(1,1)+=0.1;
        std::cout << P(1,1) <<std::endl;
        std::cout << P*f_m <<std::endl;
    }*/
}
void M2Transparent::exitCode(void) {
    robot->setEndEffForce(VM2::Zero());
}


void M2ArcCircle::entryCode(void) {
    movement_finished = false;
    goToStartPt = false;
    robot->initVelocityControl();

    //Initialise values (from network command) and sanity check
    theta_s = OWNER->global_start_angle;
    radius = OWNER->global_radius;
    centerPt = OWNER->global_center_point;
    startingPt = OWNER->global_start_point;

    for(int i=0; i<9; i++) {std::cout << "Velocity (overview) is " << ang_vel[OWNER->vel_sequence[i]] << " degree/second \n";}

    dTheta_t = ang_vel[OWNER->vel_sequence[OWNER->movement_loop]];
    OWNER->movement_loop ++;
    std::cout << "Velocity is "<< dTheta_t << " degree/second \n";

    thetaRange=80;
    ddTheta=200;
    theta = theta_s;

	//Initialise profile timing
	t_init = 3.0; //waiting time before movement starts (need to be at least 0.8 because drives have a lag...)
	t_end_accel = t_init + dTheta_t/ddTheta; //acceleration phase to reach constant angular velociy
	t_end_cstt = t_end_accel + (thetaRange-(dTheta_t*dTheta_t)/ddTheta)/dTheta_t; //constant angular velocity phase: ensure total range is theta_range
	t_end_decel = t_end_cstt + dTheta_t/ddTheta; //decelaration phase

	//Define sign of movement based on starting angle
	sign=1;
	if(theta_s>90){
		sign=-1;
		}
}
void M2ArcCircle::duringCode(void) {
    //Define velocity profile phase based on timing
    double dTheta = 0;
    VM2 dXd, Xd, dX;
    double t = elapsedTime;
    if(t<t_init)
    {
        dTheta=0;
    }
    else
    {
        if(t<t_end_accel)
        {
            //Acceleration phase
            dTheta=(t-t_init)*ddTheta;
        }
        else
        {
            if(t<=t_end_cstt)
            {
                //Constant phase
                dTheta=dTheta_t;
            }
            else
            {
                if(t<t_end_decel)
                {
                    //Deceleration phase
                    dTheta=dTheta_t-(t-t_end_cstt)*ddTheta;
                }
                else
                {
                    //Profile finished
                    dTheta=0;
                    movement_finished = true;
                }
            }
        }
    }
    dTheta*=sign;

    //Integrate to keep mobilisation angle
    theta += dTheta*dt;

    //Transform to end effector space
    //desired velocity
    dXd[0] = -radius*sin(theta*M_PI/180.)*dTheta*M_PI/180.;
    dXd[1] = radius*cos(theta*M_PI/180.)*dTheta*M_PI/180.;
    //desired position
    Xd[0] = centerPt[0]+radius*cos(theta*M_PI/180.);
    Xd[1] = centerPt[1]+radius*sin(theta*M_PI/180.);
    //PI in velocity-position
    float K=5.0;
    //dX=dXd;
    dX = dXd + K*(Xd-robot->getEndEffPosition());
    //set both 0 to calibrate force sensors
    //dX[0]=0; dX[1]=0;

    //Apply
    if(robot->isEnabled())
    {
        robot->setEndEffVelocity(dX);
    }
    else
    {
        OWNER->goToTransparentFlag = true;
    }

    if(iterations%100==1) {
        std::cout << dXd.transpose() << "  ";
        robot->printStatus();
    }

    if(movement_finished && t>t_end_decel+3) //wait three seconds
    {
        goToStartPt = true; //trigger event
    }
}
void M2ArcCircle::exitCode(void) {
    robot->setEndEffVelocity(VM2::Zero());
}


void M2Recording::entryCode(void) {
    recordingDone=false;
    recordingError=false;
    robot->initTorqueControl();
    robot->setEndEffForce(VM2::Zero());

    ForceP(0,0) = 1.3;
    ForceP(1,1) = 1.4;
    //Define Variables
    RecordingPoint=0;
}
void M2Recording::duringCode(void) {
    //Transparent force control
    VM2 f_m = robot->getInteractionForceRef();
    robot->setEndEffForce(ForceP*f_m);

	//Record stuff...
	PositionNow=robot->getEndEffPosition();
	PositionRecorded[RecordingPoint]=PositionNow;
	RecordingPoint++;

    if(iterations%100==1) {
        robot->printStatus();
    }

    // allow 10 seconds for recording
    double t = elapsedTime;
    if(t>=10){
        robot->setEndEffForce(VM2::Zero());

        ///Identify circle
	    // Fit a circle on a data point cloud using Pratt method
        // V.Pratt, "Direct least-squares fitting of algebraic surfaces",
        // Computer Graphics, Vol. 21, pages 145-152 (1987)
        //
        // Inspired from Matlab function CircleFitByPratt by Nikolai Chernov

        n = RecordingPoint-1;// number of data points
        //Debug.Log("Number of points:" + n.ToString());
        //Find centroid of data set
        for (int i=0; i<n; i++)
        {
            centroid += PositionRecorded[i];
            //std::cout << PositionRecorded[i].transpose() << " \n ";
        }
        centroid = centroid/n;
        //std::cout << centroid.transpose() << "  ";

        //computing moments(note: all moments will be normalised, i.e.divided by n)
        Mxx = 0; Myy = 0; Mxy = 0; Mxz = 0; Myz = 0; Mzz = 0;
        for (int j = 0; j < n; j++)
        {
            // centering data
            Xi = PositionRecorded[j][0] - centroid[0];
            //std::cout << PositionRecorded[j][1] << " \n ";
            Yi = PositionRecorded[j][1] - centroid[1];
            Zi = Xi * Xi + Yi * Yi;
            Mxy = Mxy + Xi * Yi;
            Mxx = Mxx + Xi * Xi;
            Myy = Myy + Yi * Yi;
            Mxz = Mxz + Xi * Zi;
            Myz = Myz + Yi * Zi;
            Mzz = Mzz + Zi * Zi;
        }
        Mxx = Mxx / n;
        Myy = Myy / n;
        Mxy = Mxy / n;
        Mxz = Mxz / n;
        Myz = Myz / n;
        Mzz = Mzz / n;

        // computing the coefficients of the characteristic polynomial
        Mz = Mxx + Myy;
        Cov_xy = Mxx * Myy - Mxy * Mxy;
        Mxz2 = Mxz * Mxz;
        Myz2 = Myz * Myz;
        A2 = 4 * Cov_xy - 3 * Mz * Mz - Mzz;
        A1 = Mzz * Mz + 4 * Cov_xy * Mz - Mxz2 - Myz2 - Mz * Mz * Mz;
        A0 = Mxz2 * Myy + Myz2 * Mxx - Mzz * Cov_xy - 2 * Mxz * Myz * Mxy + Mz * Mz * Cov_xy;
        A22 = A2 + A2;
        epsilon = 1e-12;
        ynew = 1e+20;
        IterMax = 20;
        xnew = 0;

        // Newton's method starting at x=0
        for (int iter = 1; iter <= IterMax; iter++)
        {
            yold = ynew;
            ynew = A0 + xnew * (A1 + xnew * (A2 + 4 * xnew * xnew));
            if (abs(ynew) > abs(yold))
            {
                //Debug.Log("Newton-Pratt goes wrong direction: |ynew| > |yold|");
                xnew = 0;
                break;
            }
            Dy = A1 + xnew * (A22 + 16 * xnew * xnew);
            xold = xnew;
            xnew = xold - ynew / Dy;
            if (abs((xnew - xold) / xnew) < epsilon)
                break;
            if (iter >= IterMax)
            {
                //Debug.Log("Newton-Pratt will not converge");
                xnew = 0;
            }
            if (xnew < 0)
            {
                //Debug.Log("Newton-Pratt negative root:  x=" + xnew.ToString());
                xnew = 0;
            }
        }

        //computing the circle parameters
        DET = xnew * xnew - xnew * Mz + Cov_xy;
        Center[0] = (Mxz * (Myy - xnew) - Myz * Mxy) / DET / 2 ;
        Center[1] = (Myz * (Mxx - xnew) - Mxz * Mxy) / DET / 2 ;
        radius = sqrt(Center[0]*Center[0] + Center[1]*Center[1] + Mz + 2 * xnew);
        std::cout << "Radius is " << radius << " meters \n";
        Center += centroid;//Shift to actual center
        std::cout << "Center point is ["<< Center.transpose() << "] \n";

        // Start angle defined with 0 along positive x axis, 180 along negative x
        start_angle = (atan2(PositionRecorded[0][1] - Center[1], PositionRecorded[0][0] - Center[0]) * 180.0 / M_PI);
        start_angle = (start_angle < -90)? 360 - abs(start_angle): start_angle;// Assume this is an indirect rotation(bottom left quadrant)
        std::cout << "Start angle is "<< start_angle << " degree \n";
        StartPt[0] = Center[0] + radius * cos(start_angle * M_PI / 180.0);
        StartPt[1] = Center[1] + radius * sin(start_angle * M_PI / 180.0);
        std::cout << "Start point is ["<< StartPt.transpose() << "] \n";


        /// resonable parameters
        if(radius>0.2 && radius<0.45 && StartPt[0]>=0 && StartPt[0]<=0.631 && StartPt[1]>=0 && StartPt[1]<=0.448 && abs(PositionRecorded[0][0]-PositionRecorded[n-1][0])>0.15 && abs(PositionRecorded[0][1]-PositionRecorded[n-1][1])>0.15){
            //if parameters reasonable, give them to global variables
            OWNER->global_center_point = Center;
            OWNER->global_start_point = StartPt;
            OWNER->global_radius = radius;
            OWNER->global_start_angle = start_angle;
            recordingDone=true;
        }
        else{
            recordingError=true;
        }
    }
}
void M2Recording::exitCode(void) {
    robot->setEndEffForce(VM2::Zero());
}


void M2MinJerkPosition::entryCode(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    goToNextVel=false;

    startTime=elapsedTime;
    Xi = robot->getEndEffPosition();
    Xf = OWNER->global_start_point;
    T=5; //Trajectory Time
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
    else
    {
        OWNER->goToTransparentFlag = true;
    }

    //Have we reached a point?
    if (status>=1. && iterations%100==1){
       //robot->setJointVelocity(VM2::Zero());
       std::cout << "OK. \n";

       //if (OWNER->test_loop==0 || OWNER->test_loop>=9){}
       if (OWNER->movement_loop>=1 && OWNER->movement_loop<=8){
          goToNextVel=true; //Trigger event: go to next velocity in one trial
       }
       if (OWNER->movement_loop>=9){
          goToNextVel=false;
          OWNER->movement_loop=0;
       }
    }
}
void M2MinJerkPosition::exitCode(void) {
    // std::cout << "Ready... \n";
    robot->setJointVelocity(VM2::Zero());
}


void M2CircleTest::entryCode(void) {
    testingDone=false;
    testingError=false;
    movement_finished = false;
    robot->initVelocityControl();

    //Initialise values (from network command) and sanity check
    theta_s = OWNER->global_start_angle;
    radius = OWNER->global_radius;
    centerPt = OWNER->global_center_point;
    startingPt = OWNER->global_start_point;

    dTheta_t = 10; //testing velocity 10 degree/second
    std::cout << "Velocity is "<< dTheta_t << " degree/second \n";

    thetaRange=80;
    ddTheta=200;
    theta = theta_s;

	//Initialise profile timing
	t_init = 3.0; //waiting time before movement starts (need to be at least 0.8 because drives have a lag...)
	t_end_accel = t_init + dTheta_t/ddTheta; //acceleration phase to reach constant angular velociy
	t_end_cstt = t_end_accel + (thetaRange-(dTheta_t*dTheta_t)/ddTheta)/dTheta_t; //constant angular velocity phase: ensure total range is theta_range
	t_end_decel = t_end_cstt + dTheta_t/ddTheta; //decelaration phase

	//Define sign of movement based on starting angle
	sign=1;
	if(theta_s>90){
		sign=-1;
		}
}
void M2CircleTest::duringCode(void) {
    //Define velocity profile phase based on timing
    double dTheta = 0;
    VM2 dXd, Xd, dX;
    double t = elapsedTime;
    if(t<t_init)
    {
        dTheta=0;
    }
    else
    {
        if(t<t_end_accel)
        {
            //Acceleration phase
            dTheta=(t-t_init)*ddTheta;
        }
        else
        {
            if(t<=t_end_cstt)
            {
                //Constant phase
                dTheta=dTheta_t;
            }
            else
            {
                if(t<t_end_decel)
                {
                    //Deceleration phase
                    dTheta=dTheta_t-(t-t_end_cstt)*ddTheta;
                }
                else
                {
                    //Profile finished
                    dTheta=0;
                    movement_finished = true;
                }
            }
        }
    }
    dTheta*=sign;

    //Integrate to keep mobilisation angle
    theta += dTheta*dt;

    //Transform to end effector space
    //desired velocity
    dXd[0] = -radius*sin(theta*M_PI/180.)*dTheta*M_PI/180.;
    dXd[1] = radius*cos(theta*M_PI/180.)*dTheta*M_PI/180.;
    //desired position
    Xd[0] = centerPt[0]+radius*cos(theta*M_PI/180.);
    Xd[1] = centerPt[1]+radius*sin(theta*M_PI/180.);

    //desired position reaches bound
    if(Xd[0]<0 || Xd[0]>0.631 || Xd[1]<0 || Xd[1]>0.448){
        testingError = true; //trigger event
    }

    //PI in velocity-position
    float K=5.0;
    dX = dXd + K*(Xd-robot->getEndEffPosition());

    //Apply
    if(robot->isEnabled())
    {
        robot->setEndEffVelocity(dX);
    }
    else
    {
        OWNER->goToTransparentFlag = true;
    }

    if(iterations%100==1) {
        std::cout << dXd.transpose() << "  ";
        robot->printStatus();
    }

    if(movement_finished && t>t_end_decel+3) //wait three seconds
    {
        testingDone = true; //trigger event
    }
}
void M2CircleTest::exitCode(void) {
    robot->setEndEffVelocity(VM2::Zero());

    OWNER->movement_loop = 0; //for a new trial
    /// randomly order velocity
    vector<int> vel_index_num={0,1,2,3,4,5,6,7,8};
    srand(time(0));
    random_shuffle(vel_index_num.begin(), vel_index_num.end());
    for(int i=0; i<9; i++) {OWNER->vel_sequence[i] = vel_index_num[i];}
    for(int i=0; i<9; i++) {std::cout << "Velocity sequence is " << OWNER->vel_sequence[i] << " \n";}
}


void M2ArcCircleReturn::entryCode(void) {
    robot->initVelocityControl();

    theta_s = OWNER->global_start_angle;
    radius = OWNER->global_radius;
    centerPt = OWNER->global_center_point;

    dTheta_t = 6; //Arc Return Velocity
    ddTheta=200;

    //Arc Return starting point
    finished = false;
    startingReturnPt = robot->getEndEffPosition();
    //std::cout << startingReturnPt.transpose() << " \n";
    startReturnAngle = (atan2(startingReturnPt[1] - centerPt[1], startingReturnPt[0] - centerPt[0]) * 180.0 / M_PI);
    thetaReturnRange = abs(startReturnAngle-theta_s);
    thetaReturn = startReturnAngle;
    std::cout << "Current angle is " << thetaReturn << " degree \n";

	//Initialise profile timing
	t_init = 1.0; //waiting time before movement starts (need to be at least 0.8 because drives have a lag...)
	t_end_accel = t_init + dTheta_t/ddTheta; //acceleration phase to reach constant angular velociy
	t_end_cstt = t_end_accel + (thetaReturnRange-(dTheta_t*dTheta_t)/ddTheta)/dTheta_t; //constant angular velocity phase: ensure total range is theta_range
	t_end_decel = t_end_cstt + dTheta_t/ddTheta; //decelaration phase
	//std::cout << t_end_accel << " \n";
	//std::cout << t_end_cstt << " \n";
	//std::cout << t_end_decel << " \n";

	//Define sign of movement based on starting angle
	sign=-1;
	if(theta_s>90){
		sign=1;
		}
}
void M2ArcCircleReturn::duringCode(void) {

    //Define velocity profile phase based on timing
    double dThetaReturn = 0;
    VM2 dXd, Xd, dX;
    double t = elapsedTime;
    if(t<t_init)
    {
        dThetaReturn=0;
    }
    else
    {
        if(t<t_end_accel)
        {
            //Acceleration phase
            dThetaReturn=(t-t_init)*ddTheta;
        }
        else
        {
            if(t<=t_end_cstt)
            {
                //Constant phase
                dThetaReturn=dTheta_t;
            }
            else
            {
                if(t<t_end_decel)
                {
                    //Deceleration phase
                    dThetaReturn=dTheta_t-(t-t_end_cstt)*ddTheta;
                }
                else
                {
                    //Profile finished
                    dThetaReturn=0;
                    finished = true;
                }
            }
        }
    }
    dThetaReturn*=sign;
    //std::cout << dThetaReturn << " \n";

    //Integrate to keep mobilisation angle
    thetaReturn += dThetaReturn*dt;
    //std::cout << thetaReturn << " \n";

    //Transform to end effector space
    //desired velocity
    dXd[0] = -radius*sin(thetaReturn*M_PI/180.)*dThetaReturn*M_PI/180.;
    dXd[1] = radius*cos(thetaReturn*M_PI/180.)*dThetaReturn*M_PI/180.;
    //desired position
    Xd[0] = centerPt[0]+radius*cos(thetaReturn*M_PI/180.);
    Xd[1] = centerPt[1]+radius*sin(thetaReturn*M_PI/180.);
    //PI in velocity-position
    float K=5.0;
    dX = dXd + K*(Xd-robot->getEndEffPosition());

    //Apply
    robot->setEndEffVelocity(dX);

    if(iterations%100==1) {
        std::cout << dXd.transpose() << "  ";
        robot->printStatus();
    }
}
void M2ArcCircleReturn::exitCode(void) {
    robot->setEndEffVelocity(VM2::Zero());
}
