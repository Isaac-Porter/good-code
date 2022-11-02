/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// L1                   motor         10              
// L2                   motor         9               
// L3                   motor         8               
// R1                   motor         1               
// R2                   motor         2               
// R3                   motor         3               
// Lift                 motor         4               
// Intake               motor         7               
// Controller2          controller                    
// liftClamp            digital_out   F               
// backLift1            digital_out   H               
// transmission         digital_out   E               
// backLift2            digital_out   G               
// lateralEnc           encoder       C, D            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "odometry.h"
#include "draw-field.h"
#include "string"
#include "ButtonClass.h"
#include "path.h"

using namespace vex;

competition Competition;

int auton=0;
double T=20;
int L=3;
double maxV=200;
double maxA=10;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Shooter.setStopping(coast);
  Intake.setStopping(coast);
  /*
  lcdButton b1(50,50,100,50,"WP","#123456");
  lcdButton b2(200,50,100,50,"LEFT TOWER","#123456");
  lcdButton b3(50,120,100,50,"RIGHT TOWER","#123456");
  lcdButton b4(200,120,100,50,"RIGHT+CENTER","#123456");
  lcdButton b5(350,50,100,50,"SKILLS","#123456");
  lcdButton b6(350,120,100,50,"ONLY CENTER","#123456");
  lcdButton b7(50,190,100,50,"FAST","#123456");

  while(true){
    if(b1.pressing()){
      auton=0;
      break;
    }
    if(b2.pressing()){
      auton=1;
      break;
    }
    if(b3.pressing()){
      auton=2;
      break;
    }
    if(b4.pressing()){
      auton=3;
      break;
    }
    if(b5.pressing()){
      auton=4;
      break;
    }
    if(b6.pressing()){
      auton=5;
      break;
    }
    if(b7.pressing()){
      auton=6;
      break;
    }
    wait(20,msec);
  }
  Brain.Screen.print("selected");*/
}

void straight(int p, int t){
  L1.setVelocity(p,percent);
  L2.setVelocity(p,percent);
  L3.setVelocity(p,percent);
  R1.setVelocity(p,percent);
  R2.setVelocity(p,percent);
  R3.setVelocity(p,percent);
  L1.spin(forward);
  L2.spin(forward);
  L3.spin(forward);
  R1.spin(forward);
  R2.spin(forward);
  R3.spin(forward);
  wait(t,msec);
}
void point(int p, int t){
  L1.setVelocity(-p,percent);
  L2.setVelocity(-p,percent);
  L3.setVelocity(-p,percent);
  R1.setVelocity(p,percent);
  R2.setVelocity(p,percent);
  R3.setVelocity(p,percent);
  L1.spin(forward);
  L2.spin(forward);
  L3.spin(forward);
  R1.spin(forward);
  R2.spin(forward);
  R3.spin(forward);
  wait(t,msec);
}
void stopWheels(){
  L1.stop();
  L2.stop();
  L3.stop();
  R1.stop();
  R2.stop();
  R3.stop();
}
double kv=1/200, ka=0.002, kp=0.01;
double lPos=0,plPos=0,rPos=0,prPos=0;
double lVel,rVel,idx,la,ra,leftPow,rightPow,tLVel,tRVel;

int chassis_control(){
  Path p1= Path();
  p1.add(0,0);
  p1.add(0,40);
  p1.add(20,40);
  p1.addPoints(6);
  p1.smooth(0.8,0.001);

  while(true){
    
    tLVel=p1.leftVel(idx);
    tRVel=p1.rightVel(idx);

    plPos=lPos; prPos=rPos;
    lPos=L1.position(degrees);
    rPos=R1.position(degrees);
    lVel=lPos-plPos;
    rVel=rPos-prPos;
    idx=p1.findIdx();
    la=p1.leftVel(idx+1)-tLVel;
    ra=p1.rightVel(idx+1)-tRVel;

    //double leftPow=kv*tLVel + ka*la + kp*(tLVel-lVel);
    //double rightPow=kv*tRVel + ka*ra + kp*(tRVel-rVel);

    /*
    L1.spin(forward,leftPow/1000,volt);
    L2.spin(forward,leftPow/1000,volt);
    L3.spin(forward,leftPow/1000,volt);
    R1.spin(forward,rightPow/1000,volt);
    R2.spin(forward,rightPow/1000,volt);
    R3.spin(forward,rightPow/1000,volt);
    */
    /*
    L1.setVelocity(leftPow,rpm);
    L2.setVelocity(leftPow,rpm);
    L3.setVelocity(leftPow,rpm);
    R1.setVelocity(rightPow,rpm);
    R2.setVelocity(rightPow,rpm);
    R3.setVelocity(rightPow,rpm);
    */
    L1.setVelocity(tLVel,rpm);
    L2.setVelocity(tLVel,rpm);
    L3.setVelocity(tLVel,rpm);
    R1.setVelocity(tRVel,rpm);
    R2.setVelocity(tRVel,rpm);
    R3.setVelocity(tRVel,rpm);


    L1.spin(forward);
    L2.spin(forward);
    L3.spin(forward);
    R1.spin(forward);
    R2.spin(forward);
    R3.spin(forward);
    task::sleep(10);
  }
  return 1;
}

task drawFieldTask(drawField);
task odoTask(positionTracking);
task move(chassis_control);

void autonomous(void) {
  L1.setStopping(coast);
  L2.setStopping(coast);
  L3.setStopping(coast);
  R1.setStopping(coast);
  R2.setStopping(coast);
  R3.setStopping(coast);

  /*

  std::vector<Point> pathNodes;
  pathNodes.push_back(Point(1,1));
  pathNodes.push_back(Point(1,100));
  pathNodes.push_back(Point(100,100));
  path=smooth(addPoints(pathNodes,6),0.15,0.75,0.001);
  
  task control(chassis_control);
  */
}


double modifier = 1;
double fwSpeed = 75;
int fwGear = 1;
int reversed = 1;
bool reversed_bool=false;
bool holding = false;

void buttonForward()
{                             // called when the up arrow button is pressed
  L1.setVelocity(modifier*100, percent); // gives the motors "n" velocity
  L2.setVelocity(modifier*100, percent); // and moves robot forward
  L3.setVelocity(modifier*100, percent);
  R1.setVelocity(modifier*100, percent);
  R2.setVelocity(modifier*100, percent);
  R3.setVelocity(modifier*100, percent);
}
void buttonBackward()
{                              // called when left arrow button pressed
  L1.setVelocity(-modifier*100, percent); // moves robot backward
  L2.setVelocity(-modifier*100, percent);
  L3.setVelocity(-modifier*100, percent);
  R1.setVelocity(-modifier*100, percent);
  R2.setVelocity(-modifier*100, percent);
  R3.setVelocity(-modifier*100, percent);
}
void buttonLeft()
{                              // not used
  L1.setVelocity(-modifier*100, percent); // turns robot left
  L2.setVelocity(-modifier*100, percent);
  L3.setVelocity(-modifier*100, percent);
  R1.setVelocity(modifier*100, percent);
  R2.setVelocity(modifier*100, percent);
  R3.setVelocity(modifier*100, percent);
}
void buttonRight()
{                             // not used
  L1.setVelocity(modifier*100, percent); // turns robot right
  L2.setVelocity(modifier*100, percent);
  L3.setVelocity(modifier*100, percent);
  R1.setVelocity(-modifier*100, percent);
  R2.setVelocity(-modifier*100, percent);
  R3.setVelocity(-modifier*100, percent);
}

void pewpew(){  // function that controls the loading of discs into the shooter
  int t=0;   // only runs when the flywheel is at a certain speed
  while(Controller1.ButtonR1.pressing() && Shooter.velocity(pct)>fwSpeed-1){
    loader.set(true); 
    wait(200,msec);  // actuates the piston to launch the disc
    loader.set(false);
    while(Shooter.velocity(pct)<fwSpeed-1){
      wait(10,msec);//waits until the flywheel is back up to target speed
      t++;
      if(t>50){
        break;
      }
    }
  }
  loader.set(false);
}

void change(){ //function that reverses a boolean that controls the direction of the robot
  reversed_bool=!reversed_bool;
}

void toggleBrake(){ //function that toggles the motors between braking and coasting
  if(!holding){
    L1.setStopping(hold);
    L2.setStopping(hold);
    L3.setStopping(hold);
    R1.setStopping(hold);
    R2.setStopping(hold);
    R3.setStopping(hold);
  }
  else{
    L1.setStopping(coast);
    L2.setStopping(coast);
    L3.setStopping(coast);
    R1.setStopping(coast);
    R2.setStopping(coast);
    R3.setStopping(coast);
  }
  holding=!holding;
}

void setFwSpeed(){ //cycles between 3 target speeds for the flywheel
  fwGear++;
  if (fwGear > 3) fwGear = 1;

  switch (fwGear) {
    case 1:
      fwSpeed = 75;
      break;
    case 2:
      fwSpeed = 80;
      break;
    case 3:
      fwSpeed = 85;
      break;
  }
}

int printSpeed(){ //prints information about the flywheel to the controller screen
  double ps=0;
  double pt=0;
  while(true){
    if(abs(Shooter.velocity(pct))-1>abs(ps) || abs(Shooter.velocity(pct))+1<abs(ps)){
      ps=Shooter.velocity(pct);
      Controller1.Screen.clearLine(2);
      Controller1.Screen.setCursor(2,1);
      Controller1.Screen.print(" %.2f actual",Shooter.velocity(pct));
    } 
    if(pt!=fwSpeed){
      pt=fwSpeed;
      Controller1.Screen.clearLine(1);
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print(" %.0f target",fwSpeed);
    }
    
    wait(100,msec);
  }
  return 0;
}

void launch(){  //launches the endgame catapults
  launcher.set(!launcher.value());
}

task printBrain(printSpeed);

void usercontrol(void)
{
  move.stop();
  modifier = 1;
  while (1)
  {
     
    // creates 2 variables, giving them the controller's vertical axes' values
    double leftPower = Controller1.Axis3.position(percent);
    if(abs(leftPower)<=10){
      leftPower=0;
    }
    double rightPower = Controller1.Axis2.position(percent);
    if(abs(rightPower)<=10){
      rightPower=0;
    }

    // updates "modifier", a variable created to slow down the robot
    // when certain buttons are pressed. the speed is multiplied by "modifier"
    
    if (Controller1.ButtonDown.pressing() && Controller1.ButtonB.pressing())
    {
      modifier = 0.2;
    }
    else if (Controller1.ButtonDown.pressing())
    {
      modifier = 0.6;
    }
    else if (Controller1.ButtonB.pressing())
    {
      modifier = 0.4;
    }
    else
    {
      modifier = 1;
    }
    modifier*=reversed;

    // flips the "front" of the robot, from the controllers perpective
    Controller1.ButtonA.pressed(change);
    if(!reversed_bool){
      reversed=1;
    }else{
      reversed=-1;
    }
    // call setFwSpeed to cycle through the different speeds
    Controller1.ButtonY.pressed(setFwSpeed);

    if(reversed==1){
      L1.setVelocity((leftPower) * modifier, percent);
      L2.setVelocity((leftPower) * modifier, percent);
      L3.setVelocity((leftPower) * modifier, percent);
      R1.setVelocity((rightPower) * modifier, percent);
      R2.setVelocity((rightPower) * modifier, percent);
      R3.setVelocity((rightPower) * modifier, percent);
    }else{
      L1.setVelocity((rightPower) * modifier, percent);
      L2.setVelocity((rightPower) * modifier, percent);
      L3.setVelocity((rightPower) * modifier, percent);
      R1.setVelocity((leftPower) * modifier, percent);
      R2.setVelocity((leftPower) * modifier, percent);
      R3.setVelocity((leftPower) * modifier, percent);
    }

    // calls the functions created earlier when their respective buttons are pressed
    if(Controller1.ButtonUp.pressing()){
      buttonForward();
    }
    if(Controller1.ButtonLeft.pressing()){
      buttonBackward();
    }

    // runs the flywheel at the target speed when R2 is pressed
    if(Controller1.ButtonR2.pressing()){
      Shooter.setVelocity(fwSpeed,pct);
    }else{
      Shooter.setVelocity(0,pct);
    }
    // calls the loading function
    Controller1.ButtonR1.pressed(pewpew);

    // calls the endgame launch function
    Controller1.ButtonX.pressed(launch);
    
    // controls the intake using the left bumpers
    if(Controller1.ButtonL1.pressing()){
      Intake.setVelocity(100,pct);
    }else if(Controller1.ButtonL2.pressing()){
      Intake.setVelocity(-100,pct);
    }else{
      Intake.setVelocity(0,pct);
    }
    

    // calls the toggle brake function
    Controller1.ButtonRight.pressed(toggleBrake);

    // spins all of the motors
    L1.spin(forward);
    L2.spin(forward);
    L3.spin(forward);
    R1.spin(forward);
    R2.spin(forward);
    R3.spin(forward);
    Intake.spin(forward);
    Shooter.spin(forward);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
