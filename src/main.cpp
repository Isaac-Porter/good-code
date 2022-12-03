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

using namespace vex;

competition Competition;

task drawFieldTask(drawField);
task odoTask(positionTracking);

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
  
  launcher2.set(!launcher2.value());
  launcher1.set(!launcher1.value());
  
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


double kp=10,ki=0.01,kd=10;
double integral=0,derivative=0;
double error=0,prevError=0,target=0,output=0,input=0;

double kpt=15,kit=0.01,kdt=10;
double tintegral=0,tderivative=0;
double terror=0,tprevError=0,ttarget=0, toutput=0, tinput=0;

bool atTarget=false;
bool atAngle=false;

bool turning=true;

double pidLim=12000;

int pid(){
  //Controller1.Screen.clearScreen();
  L1.setPosition(0,degrees);
  L2.setPosition(0,degrees);
  L3.setPosition(0,degrees);
  R1.setPosition(0,degrees);
  R2.setPosition(0,degrees);
  R3.setPosition(0,degrees);
  
  while(true){
    input=(L1.position(degrees)+L2.position(degrees)+L3.position(degrees)+R1.position(degrees)+R2.position(degrees)+R3.position(degrees))/6;
    error=target-input;
    if(prevError==0 && error!=0){
      prevError=error;
    }
    integral+=error;
    derivative=error-prevError;
    prevError=error;
    
    output=kp*error+ki*integral+kd*derivative;

    if(turning){
      tinput=(L1.position(degrees)+L2.position(degrees)+L3.position(degrees)-R1.position(degrees)-R2.position(degrees)-R3.position(degrees))/6;
      terror=ttarget-tinput;
      if(tprevError==0 && terror!=0){
        tprevError=error;
      }
      tintegral+=terror;
      tderivative=terror-tprevError;
      tprevError=terror;
      
      toutput=kpt*terror+kit*tintegral+kdt*tderivative;
    }else{
      toutput=0;
    }

    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.clearLine(3);
    Controller1.Screen.print("%f",Shooter.velocity(pct));

    if(output>pidLim){
      output=pidLim;
    }else if(output<-pidLim){
      output=-pidLim;
    }
    if(toutput>pidLim){
      output=pidLim;
    }else if(toutput<-pidLim){
      output=-pidLim;
    }
    
    R1.spin(forward,(output-toutput)/1000,volt);
    R2.spin(forward,(output-toutput)/1000,volt);
    R3.spin(forward,(output-toutput)/1000,volt);
    L1.spin(forward,(output+toutput)/1000,volt);
    L2.spin(forward,(output+toutput)/1000,volt);
    L3.spin(forward,(output+toutput)/1000,volt);

    wait(10,msec);

    if(derivative<0.5 && derivative>-0.5 && error<100 && error>-100){
      atTarget=true;
    }else{
      atTarget=false;
    }
    if(tderivative<0.5 && tderivative>-0.5 && terror<50 && terror >-50){
      atAngle=true;
    }else{
      atAngle=false;
    }

    g_target=ttarget;
  }
  return 1;
}
void resetPID(){
  L1.setPosition(0,degrees);
  L2.setPosition(0,degrees);
  L3.setPosition(0,degrees);
  R1.setPosition(0,degrees);
  R2.setPosition(0,degrees);
  R3.setPosition(0,degrees);
  error=0;
  terror=0;
  prevError=0;
  tprevError=0;
  integral=0;
  tintegral=0;
  target=0;
  ttarget=0;
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
  wait(200,msec);
  while((tderivative>0.5 || tderivative<-0.5)/* && (error>200 || error <-200)*/){
    wait(3,msec);
  }
}

void pewpew_auto(int d, int p){  // function that controls the loading of discs into the shooter
  //Shooter.spin(forward,100,pct);
  int t=0;
  wait(200,msec);
  for(int i=0; i<d; i++){
    t=0;
    while(Shooter.velocity(pct)<p || t<20){
      wait(10,msec);//waits until the flywheel is back up to target speed
      t++;
    }
    loader.set(true); 
    wait(200,msec);  // actuates the piston to launch the disc
    loader.set(false);
  }
  Shooter.stop();
}

void left_side(){
  target=150;
  Intake.spin(forward,-100,pct);
  Shooter.spin(forward,87,pct);
  wait(800,msec);
  Intake.stop();

  target=-170;
  ew();
  wait(300,msec);

  ttarget=-170;
  wait(500,msec);
  pewpew_auto(2,85);
  wait(400,msec);

  resetPID();

  ttarget=-710;
  tw();
  
  resetPID();
  target=2400;
  pidLim=7500;
  Intake.spin(forward,100,pct);
  wait(800,msec);
  pidLim=2000;
  pw();
  pidLim=12000;

  ttarget=570;
  Shooter.spin(forward,85,pct);
  pewpew_auto(3,85);
}

void win_point(){
  target=170;
  Intake.spin(forward,-100,pct);
  Shooter.spin(forward,86,pct);
  wait(800,msec);
  Intake.stop();

  target=-200;
  ew();
  wait(300,msec);

  ttarget=-190;
  wait(500,msec);
  pewpew_auto(2,85);
  wait(400,msec);

  resetPID();
  ttarget=350;
  wait(1000,msec);

  resetPID();
  target=-7200;
  wait(300,msec);
  ew();

  ttarget=1400;
  tw();

  resetPID();
  Intake.spin(forward,100,pct);
  target=1500;
  wait(300,msec);
  pw();

  ttarget=400;
  wait(300,msec);
  etw();

  resetPID();
  target=500;
  Intake.spin(forward,-100,pct);
  wait(4500,msec);
  Intake.stop();
  
}

double g_target=0;
//double g_input=0;
task pid_task(pid);
//task graphthePID(tgraph);
//task move(chassis_control);

void autonomous(void) {
  pid_task.stop();
  task pid_task(pid);
  L1.setStopping(coast);
  L2.setStopping(coast);
  L3.setStopping(coast);
  R1.setStopping(coast);
  R2.setStopping(coast);
  R3.setStopping(coast);
  resetPID();


  //turning=false;
  
  left_side();


  pid_task.stop();
  

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
  while(Shooter.velocity(pct)<fwSpeed-1){
      wait(10,msec);//waits until the flywheel is back up to target speed
      t++;
      if(t>100){
        break;
      }
    }
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

void launch1(){  //launches the endgame catapults
  launcher1.set(!launcher1.value());
}
void launch2(){  //launches the endgame catapults
  launcher2.set(!launcher2.value());
}


void usercontrol(void)
{
  task printBrain(printSpeed);
  pid_task.stop();
  //move.stop();
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
    //Controller1.ButtonA.pressed(change);
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
    Controller1.ButtonX.pressed(launch1);
    Controller1.ButtonA.pressed(launch2);
    
    // controls the intake using the left bumpers
    if(Controller1.ButtonL1.pressing()){
      Intake.setVelocity(100,pct);
    }else if(Controller1.ButtonL2.pressing()){
      Intake.setVelocity(-100,pct);
    }else{
      Intake.setVelocity(0,pct);
    }
    

    // calls the toggle ke function
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
