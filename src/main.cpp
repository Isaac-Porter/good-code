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
#include "chassis_control.h"
#include "graph-stuff.h"
#include "pid.h"

using namespace vex;

competition Competition;

task drawFieldTask(drawField);
task odoTask(positionTracking);
//task graphTask(fgraph);
task pid_task(pid);
task fwTask(flywheel_pid);
double g_target=420;

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
  pid_task.suspend();
  fwTask.suspend();
  
  
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

void pw(){//function that will wait until the robot is at its target
  int t=0;
  wait(300,msec);
  while(!atTarget){
    wait(10,msec);
    t++;
    if(t>1000){
      break;
    }
  }
}
void tw(){//function that will wait until the robotis at its turning target
  int t=0;
  wait(300,msec);
  while(!atAngle){
    wait(10,msec);
    t++;
    if(t>1000){
      break;
    }
  };
}
void ew(){
  wait(200,msec);
  while((derivative>0.5 || derivative<-0.5)/* && (error>200 || error <-200)*/){
    wait(3,msec);
  }
}
void etw(){
  wait(400,msec);
  while((tderivative>0.5 || tderivative<-0.5)/* && (error>200 || error <-200)*/){
    wait(3,msec);
  }
}
void intakeThingyWait(){
  Intake.spin(forward,100,pct);
  wait(200,msec);
  while((derivative>0.5 || derivative<-0.5)/* && (error>200 || error <-200)*/){
    if(Intake.velocity(pct)<10){
      Intake.spin(reverse, 100, pct);
      wait(200,msec);
      Intake.spin(forward,100,pct);
      wait(500,msec);
    }
    wait(3,msec);
  }
}
void intakeThingyWait2(){
  Intake.spin(forward,100,pct);
  wait(200,msec);
  while((derivative>1 || derivative<-1)/* && (error>200 || error <-200)*/){
    if(Intake.velocity(pct)<10){
      Intake.spin(reverse, 100, pct);
      wait(200,msec);
      Intake.spin(forward,100,pct);
      wait(500,msec);
    }
    wait(3,msec);
  }
}
int intakeThingy(){
  while(true){
    Intake.spin(forward,100,pct);
    wait(1500,msec);
    Intake.spin(reverse,100,pct);
    wait(250,msec);
  }
  return 1;
}


void pewpew_auto(int d, double p){  // function that controls the loading of discs into the shooter
  //Shooter.spin(forward,100,pct);
  int t=0;
  for(int i=0; i<d; i++){
    t=0;
    while(Shooter.velocity(pct)<p+1 || t<20){
      wait(10,msec);//waits until the flywheel is back up to target speed
      t++;
    }
    loader.set(true); 
    wait(200,msec);  // actuates the piston to launch the disc
    loader.set(false);
  }
  //Shooter.stop();
}
void pewpew2(int d){
  int t=0;
  for(int i=0; i<d; i++){
    t=0;
    while(std::abs(ferror1)>1 || t<30){
      wait(10,msec);//waits until the flywheel is back up to target speed
      t++;
    }
    loader.set(true); 
    wait(200,msec);  // actuates the piston to launch the disc
    loader.set(false);
  }
}
/*
void stopFW(){
  for(int i=12; i>-1; i-=0.5){
    Shooter.spin(forward,i,volt);
    wait(20,msec);
  }
  Shooter.stop();
}*/

void left_side(){
  turning=true;
  target=100;
  ttarget=-250;
  Intake.spin(forward,-100,pct);
  wait(600,msec);
  Intake.stop();
  
  /*
  //ttarget=0;
  target=-500;
  ew();
  wait(300,msec);
  turning=true;
  
  //ttarget=-200;
  //Shooter.spin(forward,100,pct);
  //pewpew_auto(2,100);
  //wait(400,msec);
  

  ttarget=-870;
  etw();
  wait(200,msec);

  target=900;
  pidLim=4000;
  Intake.spin(forward,100,pct);
  ew();
  wait(1000,msec);
  target=1300;
  ew();
  wait(1000,msec);
  target=1600;
  ew();
  //intakeThingyWait();

  pidLim=12000;
  resetPID();

  //task intakeTast(intakeThingy);

  ttarget=670;//606
  Shooter.spin(forward,90,pct);
  pewpew_auto(1,90);
  wait(500,msec);
  pewpew_auto(1,90);
  wait(500,msec);
  pewpew_auto(1,90);
  //intakeTast.stop();
  */
}

void right_side(){
  /* complicated path
  turning=true;
  target=1950;
  Intake.spin(forward,100,pct);
  wait(500,msec);
  ew();
  turning=true;
  ttarget=-1025;
  tw();
  Intake.spin(reverse,100,pct);
  wait(200,msec);
  Intake.stop();
  int v=93;
  Shooter.spin(forward,v,pct);
  pewpew_auto(1,v);
  wait(500,msec);
  pewpew_auto(1,v);
  wait(500,msec);
  pewpew_auto(1,v);
  Shooter.stop();
  */
  


  //simple path
  Shooter.spin(forward,60,pct);
  pewpew_auto(1,50);
  wait(1000,msec);
  pewpew_auto(1,50);
  Shooter.stop();
  target=1000;
  wait(500,msec);
  ew();
  resetPID();
  ttarget=750;
  etw();
  target=450;
  Intake.spin(forward,-100,pct);
  wait(1000,msec);
  Intake.stop();
  wait(1000,msec);
  target=200;
  pw();
  /*
  resetPID();
  ttarget=800;
  tw();
  Intake.spin(forward,100,pct);
  target=1200;
  ew();
  */
}

void win_point(){
  // move forward, turn slightly, and spin roller
  // target=100;
  turning=true;
  shooting=false;
  ttarget=-250;
  Intake.spin(forward,-100,pct);
  wait(500,msec);
  Intake.stop();

  // turn back slighlty and back away from roller
  target=-235;
  ttarget=0;
  wait(500,msec);

  // turn to face 3-stack of discs
  ttarget=-885;
  etw();

  // move forward and start intaking
  Intake.spin(forward,100,pct);
  target=2100;
  wait(500,msec);

  // start revving flywheel
  shooting=true;
  ftarget=87;

  // continue moving forward to intake 3-stack of discs and align with high goal
  pidLim=6000;
  intakeThingyWait();
  pidLim=12000;

  // turn to shoot at high goal
  ttarget=-290;
  wait(300,msec);
  etw();

  // shoot 3 dics
  pewpew2(3);
  shooting=false;

  // turn to face line of 3 discs
  ttarget=-925;
  wait(300,msec);
  etw();

  // move forward and intake line of 3 discs
  //wait(300,msec);
  target=6650;
  wait(300,msec);
  intakeThingyWait2();

  // turn slightly to adjust for roller
  // resetPID();
  // ttarget=-250;
  // etw();

  // drive forward
  // target=900;
  // pw();

  // turn towards roller
  ttarget=-525;
  etw();

  // move forwards and start intake (for roller mech)
  resetPID();
  target=700;
  Intake.spin(forward,-100,pct);

  // start revving flywheel
  ftarget=100;
  shooting=true;

  // spin roller
  ew();
  resetPID();
  wait(600,msec);
  Intake.stop();

  // turn to shoot at high goal
  //ttarget=100;
  //wait(400,msec);

  // shoot 3 discs
  pewpew2(3);
  shooting=false;
  
}

void skill(){
  turning=true;
  target=80;
  ttarget=-250;
  Intake.spin(forward,-100,pct);
  wait(600,msec);
  Intake.stop();
  wait(500,msec);
  resetPID();
  target=-1100;
  pw();
  ttarget=-870;
  tw();
  launcher1.set(true);
  launcher2.set(true);
  wait(1000,msec);
  resetPID();
  target=-1200;
  pw();
}

void test(){
  target=1500;
  wait(500,msec);
  pw();
}

void poop(){
  Shooter.spin(forward,40,pct);
  pewpew_auto(1,40);
  Shooter.spin(forward,40,pct);
  wait(1000,msec);
  pewpew_auto(1,40);
}

//task move(chassis_control);

void autonomous(void) {
  pid_task.resume();
  fwTask.resume();
  L1.setStopping(coast);
  L2.setStopping(coast);
  L3.setStopping(coast);
  R1.setStopping(coast);
  R2.setStopping(coast);
  R3.setStopping(coast);
  resetPID();
  
  win_point();

  pid_task.stop();
  fwenable=false;
  

  /*
  std::vector<Point> pathNodes;
  pathNodes.push_back(Point(1,1));
  pathNodes.push_back(Point(1,100));
  pathNodes.push_back(Point(100,100));
  path=smooth(addPoints(pathNodes,6),0.15,0.75,0.001);
  task control(chassis_control);
  */
}

double fwSpeed = 70;
bool revving=false;
double modifier = 1;
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
  int t=0;
  while(Controller1.ButtonR1.pressing()){
    t=0;
    while(Shooter.velocity(pct)<fwSpeed-0.5 || t<40){
      wait(10,msec);//waits until the flywheel is back up to target speed
      t++;
      if(!Controller1.ButtonR1.pressing()){
        break;
      }
    }
    //blocker.set(false);
    loader.set(true); 
    wait(200,msec);  // actuates the piston to launch the disc
    loader.set(false);
  }
  //blocker.set(true);
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
  switch(fwGear){
    case 1: fwSpeed=70;
            break;
    case 2: fwSpeed=75;
            break;
    default:
            fwSpeed=65;
            fwGear=0;
  }
  g_target=fwSpeed;
}

int printSpeed(){ //prints information about the flywheel to the controller screen
  double ps=0;
  double pt=0;
  while(true){
    if(std::abs(Shooter.velocity(pct))-1>std::abs(ps) || std::abs(Shooter.velocity(pct))+1<std::abs(ps)){
      ps=Shooter.velocity(pct);
      Controller1.Screen.clearLine(2);
      Controller1.Screen.setCursor(2,1);
      Controller1.Screen.print(" %.2f actual",Shooter.velocity(pct));
    } 
    if(true){
      pt=fwSpeed;
      Controller1.Screen.clearLine(1);
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print(" %.0f target",fwSpeed);
    }
    
    wait(100,msec);
  }
  return 0;
}

void usercontrol(void)
{
  pid_task.stop();
  fwenable=false;
  //move.stop();
  Controller1.ButtonY.pressed(setFwSpeed);
  // calls the loading function
  Controller1.ButtonR1.pressed(pewpew);
  task printController(printSpeed);
  blocker.set(true);
  modifier = 1;
  while (1)
  {
    // creates 2 variables, giving them the controller's vertical axes' values
    double leftPower = Controller1.Axis3.position(percent);
    if(std::abs(leftPower)<=10){
      leftPower=0;
    }
    double rightPower = Controller1.Axis2.position(percent);
    if(std::abs(rightPower)<=10){
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
    //Controller1.ButtonA.pressed(change);
    if(!reversed_bool){
      reversed=1;
    }else{
      reversed=-1;
    }
    // call setFwSpeed to cycle through the different speeds
    

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
    /*
    if(Controller1.ButtonUp.pressing()){
      buttonForward();
    }
    if(Controller1.ButtonLeft.pressing()){
      buttonBackward();
    }*/

    // runs the flywheel at the target speed when R2 is pressed
    if(Controller1.ButtonR2.pressing()){
      Shooter.setVelocity(fwSpeed,pct);
      revving=true;
    }else{
      revving=false;
      Shooter.setVelocity(0,pct);
      //blocker.set(true);
    }
    
    // controls the intake using the left bumpers
    if(Controller1.ButtonL1.pressing()){
      Intake.setVelocity(100,pct);
    }else if(Controller1.ButtonL2.pressing()){
      Intake.setVelocity(-100,pct);
    }else{
      Intake.setVelocity(0,pct);
    }
    

    // calls the toggle ke function
    //Controller1.ButtonRight.pressed(toggleBrake);
    if(Controller1.ButtonRight.pressing() && !fwenable){
      Shooter.setVelocity(-100,pct);
    }

    if(Controller1.ButtonX.pressing() && Controller1.ButtonUp.pressing()){
      launcher1.set(true);
      launcher2.set(true);
    }

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
