#include "stepper.h"
#include <math.h>
#include <stdio.h>


typedef struct {
    double x;
    double y;
} PointCartisian;

typedef struct {
    double r;
    double theta;
} PointPolar;

typedef struct {
    PointCartisian LT;
    PointCartisian RT;
    PointCartisian LB;
    PointCartisian RB;
} RobotWheels;

typedef struct {
    double x;
    double y;
    double theta;
} Input;
double pi = 3.14159265358979323846;
StepperMotor motor1;
StepperMotor motor2;
StepperMotor motor3;
StepperMotor motor4;
const int pins[] = {23, 19, 22, 18};//
const int pins2[] = {5, 16, 17, 4};
const int pins3[] = {32, 25, 33, 26};
const int pins4[] = {27, 13, 14, 21};
const int stepsPerRevolution = 200;
const double wheelDiameter = 80.0;



PointCartisian rotate(PointCartisian wheel, double theta);
PointCartisian toCartesian(PointPolar p);
PointPolar toPolar(PointCartisian p);
PointCartisian translate(PointCartisian wheel, PointCartisian input);
RobotWheels moveRobot(RobotWheels robot, Input move);
PointCartisian convToUnitVector(PointCartisian start, PointCartisian end);
bool sign (double x);
int toSteps(double inches, double wheelDiameter,int stepsPerRevolution);
double totalmove(PointCartisian start, PointCartisian end);





PointCartisian toCartesian(PointPolar p) {
    return (PointCartisian){p.r * cos(p.theta), p.r * sin(p.theta)};
}

PointPolar toPolar(PointCartisian p) {
    return (PointPolar){sqrt(p.x * p.x + p.y * p.y), atan2(p.y, p.x)};
}




PointCartisian rotate(PointCartisian wheel, double theta){
    PointPolar rotated = toPolar(wheel);
    rotated.theta += theta;
    PointCartisian newCoordinates = toCartesian(rotated);
    return newCoordinates;
}

PointCartisian translate(PointCartisian wheel, PointCartisian input){
    PointCartisian newCoordinates = {wheel.x + input.x, wheel.y + input.y};
    return newCoordinates;
}

RobotWheels moveRobot(RobotWheels robot, Input move){
    RobotWheels newRobot;
    newRobot.LT = rotate(robot.LT, move.theta);
    printf("newRobot.LT: %f, %f\n", newRobot.LT.x, newRobot.LT.y);
    newRobot.RT = rotate(robot.RT, move.theta);
    printf("newRobot.RT: %f, %f\n", newRobot.RT.x, newRobot.RT.y);
    newRobot.LB = rotate(robot.LB, move.theta);
    printf("newRobot.LB: %f, %f\n", newRobot.LB.x, newRobot.LB.y);
    newRobot.RB = rotate(robot.RB, move.theta);
    printf("newRobot.RB: %f, %f\n", newRobot.RB.x, newRobot.RB.y);
    newRobot.LT = translate(newRobot.LT, (PointCartisian){move.x, move.y});
    newRobot.RT = translate(newRobot.RT, (PointCartisian){move.x, move.y});
    newRobot.LB = translate(newRobot.LB, (PointCartisian){move.x, move.y});
    newRobot.RB = translate(newRobot.RB, (PointCartisian){move.x, move.y});
    return newRobot;
}

    
PointCartisian convToUnitVector(PointCartisian start, PointCartisian end){
    PointCartisian p = {end.x - start.x, end.y - start.y};
    // double direction_x = sign(end.x);
    // double direction_y = sign(end.y);
    if(p.x < 0.001 && p.x > -0.001 && p.y < 0.001 && p.y > -0.001){
        PointCartisian unitVector = {0, 0};
        return unitVector;
    }
    double magnitude = sqrt(p.x * p.x + p.y * p.y);
    PointCartisian unitVector = {p.x / magnitude, p.y / magnitude};
    return unitVector;
}
int toSteps(double mm, double wheelDiameter,int stepsPerRevolution){
    double circumference = 3.14159265358979323846 * wheelDiameter;
    return (mm / circumference) * stepsPerRevolution;
}

double totalmove(PointCartisian start, PointCartisian end){
    double x = end.x - start.x;
    double y = end.y - start.y;
    return sqrt(x*x + y*y);
}
bool sign(double value){
    if(value < 0){
        return false;
    }
    return true;
}
  
void app_main() {
    
    initStepperTimer();

    RobotWheels robot = {{-100, 52}, {100, 52}, {-100, -52}, {100, -52}};
    
    PointCartisian LT = robot.LT;
    PointCartisian RT = robot.RT;
    PointCartisian LB = robot.LB;
    PointCartisian RB = robot.RB;

    double input[3] = {-1000,0,0};//use radians 
    Input move = {input[0], input[1], input[2]};
    Input moveSmall = {move.x / 100000, move.y / 100000, move.theta / 100000};
    robot = moveRobot(robot, moveSmall);
    printf("robot: %f, %f, %f, %f, %f, %f, %f, %f\n", robot.LT.x, robot.LT.y, robot.RT.x, robot.RT.y, robot.LB.x, robot.LB.y, robot.RB.x, robot.RB.y);
    printf("moveSmall: %f, %f, %f\n", moveSmall.x, moveSmall.y, moveSmall.theta);

    PointCartisian LTvec = convToUnitVector(LT, robot.LT);
    PointCartisian RTvec = convToUnitVector(RT, robot.RT);
    PointCartisian LBvec = convToUnitVector(LB, robot.LB);
    PointCartisian RBvec = convToUnitVector(RB, robot.RB);
    
    
    int LTsteps = toSteps(totalmove(LT, robot.LT)* 100000, wheelDiameter, 200) ;
    int RTsteps = toSteps(totalmove(RT, robot.RT)* 100000, wheelDiameter, 200) ;
    int LBsteps = toSteps(totalmove(LB, robot.LB)* 100000, wheelDiameter, 200) ;
    int RBsteps = toSteps(totalmove(RB, robot.RB)* 100000, wheelDiameter, 200) ;

    double LTspeed = (LTvec.y+LTvec.x) / 2;
    double RTspeed = (RTvec.y-RTvec.x) / 2;
    double LBspeed = (LBvec.y-LBvec.x) / 2;
    double RBspeed = (RBvec.y+RBvec.x) / 2;
    
    LTspeed = LTspeed * 500;
    RTspeed = RTspeed * 500;
    LBspeed = LBspeed * 500;
    RBspeed = RBspeed * 500;
   
    printf("LTsteps: %d, RTsteps: %d, LBsteps: %d, RBsteps: %d\n", LTsteps, RTsteps, LBsteps, RBsteps);
    printf("LTvec.x: %f, LTvec.y: %f, RTvec.x: %f, RTvec.y: %f, LBvec.x: %f, LBvec.y: %f, RBvec.x: %f, RBvec.y: %f\n", LTvec.x, LTvec.y, RTvec.x, RTvec.y, LBvec.x, LBvec.y, RBvec.x, RBvec.y);
    printf("LTvec: %f, %f, RTvec: %f, %f, LBvec: %f, %f, RBvec: %f, %f\n", LTvec.x, LTvec.y, RTvec.x, RTvec.y, LBvec.x, LBvec.y, RBvec.x, RBvec.y);
    printf("LTvec: %f, %f, RTvec: %f, %f, LBvec: %f, %f, RBvec: %f, %f\n", LTvec.x, LTvec.y, RTvec.x, RTvec.y, LBvec.x, LBvec.y, RBvec.x, RBvec.y);


    // Set up motor movement
    setUpMovement(&motor1, pins, fabs(LTspeed), (int)abs(LTsteps),  !sign(LTspeed));
    setUpMovement(&motor2, pins2, fabs(RTspeed), (int)abs(RTsteps), sign(RTspeed));
    setUpMovement(&motor3, pins3, fabs(LBspeed), (int)abs(LBsteps), !sign(LBspeed));
    setUpMovement(&motor4, pins4, fabs(RBspeed), (int)abs(RBsteps), sign(RBspeed));
    // Main loop

    printf("LTspeed: %f, RTspeed: %f, LBspeed: %f, RBspeed: %f\n", LTspeed, RTspeed, LBspeed, RBspeed);
    printf("LTsteps: %d, RTsteps: %d, LBsteps: %d, RBsteps: %d\n", LTsteps, RTsteps, LBsteps, RBsteps);
    printf("LTvec.x: %f, LTvec.y: %f, RTvec.x: %f, RTvec.y: %f, LBvec.x: %f, LBvec.y: %f, RBvec.x: %f, RBvec.y: %f\n", LTvec.x, LTvec.y, RTvec.x, RTvec.y, LBvec.x, LBvec.y, RBvec.x, RBvec.y);
    printf("LTvec: %f, %f, RTvec: %f, %f, LBvec: %f, %f, RBvec: %f, %f\n", LTvec.x, LTvec.y, RTvec.x, RTvec.y, LBvec.x, LBvec.y, RBvec.x, RBvec.y);
    printf("LTvec: %f, %f, RTvec: %f, %f, LBvec: %f, %f, RBvec: %f, %f\n", LTvec.x, LTvec.y, RTvec.x, RTvec.y, LBvec.x, LBvec.y, RBvec.x, RBvec.y);
    while (1) {
        if (isTimerTickReady()) {
            stepMotor(&motor1);
            stepMotor(&motor2);
            stepMotor(&motor3);
            stepMotor(&motor4);
        }
    }
}