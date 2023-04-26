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

//task drawFieldTask(drawField);
task odoTask(positionTracking);
task graphTask(fgraph);
task pid_task(pid);
task fwTask(flywheel_pid);

int auton=0;
double T=20;
int L=3;
double maxV=200;
double maxA=10;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  blocker.set(true);
  resetPID();
  vexcodeInit();
  Shooter.setStopping(coast);
  Intake.setStopping(coast);
  pid_task.suspend();
  fwTask.suspend();
  dinglebopper.set(false);
  
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
void newWait(){
  wait(400,msec);
  while(error>0){
    wait(3,msec);
  }
}
void newtWait(){
  wait(400,msec);
  while(terror>0){
    wait(3,msec);
  }
}
void intakeThingyWait(){
  Intake.spin(forward,100,pct);
  wait(200,msec);
  while((derivative>0.5 || derivative<-0.5)/* && (error>200 || error <-200)*/){
    if(Intake.velocity(pct)<10){
      Intake.spin(reverse, 100, pct);
      wait(400,msec);
      Intake.spin(forward,100,pct);
      wait(500,msec);
    }
    wait(3,msec);
  }
}
void intakeThingyWait2(){
  Intake.spin(forward,100,pct);
  wait(500,msec);
  while((derivative>5 || derivative<-5)/* && (error>200 || error <-200)*/){
    if(Intake.velocity(pct)<10){
      Intake.spin(reverse, 100, pct);
      wait(400,msec);
      Intake.spin(forward,100,pct);
      wait(500,msec);
    }
    wait(3,msec);
  }
}
int intakeThingy(){
  double pos=0;
  Intake.spin(fwd,100,pct);
  while(true){
    if(Intake.velocity(pct)<5){
      wait(500,msec);
      pos=Intake.position(rev);
      if(Intake.velocity(pct)<5 && std::abs(Intake.position(rev)-pos)<1){
        Intake.spin(reverse, 100, pct);
        wait(400,msec);
        Intake.spin(forward,100,pct);
        wait(000,msec);
      }
    }
    wait(3,msec);
  }
  return 1;
}int intakeThingy2(){
  wait(500,msec);
  Intake.spin(fwd,-100,pct);
  wait(300,msec);
  Intake.spin(fwd,100,pct);
  return 1;
}


void pewpew_auto(int d, double p){  // function that controls the loading of discs into the shooter
  //Shooter.spin(forward,100,pct);
  int t=0;
  blocker.set(false);
  for(int i=0; i<d; i++){
    t=0;
    while(Shooter.velocity(pct)<p+1 || t<20){
      wait(10,msec);//waits until the flywheel is back up to target speed
      t++;
    }
    wait(300,msec);
    loader.set(true); 
    wait(200,msec);  // actuates the piston to launch the disc
    loader.set(false);
  }
  blocker.set(true);
  //Shooter.stop();
}
void pewpew2(int d){
  int t=0;
  bool retry=false;
  blocker.set(false);
  for(int i=0; i<d; i++){
    // if(!retry){
      t=0;
    // }
    
    while(((std::abs(ferror1)>0.25 && std::abs(fprevError)>0.25) || t<60) && t<1000){
      wait(10,msec);//waits until the flywheel is back up to target speed
      t++;
    }
    // wait(50,msec);
    //if(std::abs(ferror1)<1){
    shot=true;
    loader.set(true); 
    wait(200,msec);  // actuates the piston to launch the disc
    loader.set(false);
    retry=false;
    // //}else{
    //   retry=true;
    //   i--;
    // }
  }
  blocker.set(true);
}
void pewpew3(int d){
  int t=0;
  bool retry=false;
  blocker.set(false);
  for(int i=0; i<d; i++){
    if(!retry){
      t=0;
    }
    t=0;
    while(/*(Shooter.velocity(pct)<100 || t<100) &&*/ t<100){
      wait(10,msec);//waits until the flywheel is back up to target speed
      t++;
    }
    wait(50,msec);
    if(/*Shooter.velocity(pct)>100*/true){
      loader.set(true); 
      wait(200,msec);  // actuates the piston to launch the disc
      loader.set(false);
      retry=false;
      shot=true;
    }else{
      retry=true;
      i--;
    }
  }
  blocker.set(true);
}
void pewpew4(int d){
  int t=0;
  bool retry=false;
  blocker.set(false);
  for(int i=0; i<d; i++){
    // if(!retry){
      t=0;
    // }
    
    while((std::abs(ferror1)>1 || t<40) && t<8000 ){
      wait(10,msec);//waits until the flywheel is back up to target speed
      t++;
    }
    // wait(50,msec);
    //if(std::abs(ferror1)<1){
    loader.set(true); 
    wait(200,msec);  // actuates the piston to launch the disc
    loader.set(false);
    retry=false;
    shot=true;
    // //}else{
    //   retry=true;
    //   i--;
    // }
  }
  blocker.set(true);
}
void pewpew5(int d){
  int t=0;
  bool retry=false;
  blocker.set(false);
  for(int i=0; i<d; i++){
    if(!retry){
      t=0;
    }
    
    while((std::abs(ferror1)>1 || t<40 || std::abs(fderivative)>1) && t<1000){
      wait(10,msec);//waits until the flywheel is back up to target speed
      t++;
    }
    wait(50,msec);
    if(std::abs(ferror1)>1 || std::abs(fderivative)>1){
      shot=true;
      loader.set(true); 
      wait(200,msec);  // actuates the piston to launch the disc
      loader.set(false);
      retry=false;
    }else{
      retry=true;
      i--;
    }
  }
  blocker.set(true);
}


void left_side_old(){
  //start spinning the flywheel
  //Shooter.spin(forward,100,pct);
  Shooter.spin(fwd,13,volt);
  pewpew3(2);

  target=400;
  Intake.spin(forward,-100,pct);
  wait(800,msec);
  Intake.stop();
  target=0;
  ew();

  // // move forward, turn slightly, and spin roller
  // turning=true;
  // ttarget=-250;
  // Intake.spin(forward,-100,pct);
  // wait(600,msec);
  // Intake.stop();

  // // turn back slighlty and back away from roller
  // target=-240;
  // ttarget=-120;
  // wait(500,msec);

  //turn towards the stack of 3 discs
  ttarget=-835;
  etw();
  wait(200,msec);

  //start spinning the flywheel again for the next shots
  shooting=true;
  ftarget=87;
  
  //moves forward slowly and intakes the discs
  target=1750+250;
  pidLim=9000;

  // Intake.spin(forward,100,pct);
  wait(500,msec);
  pidLim=5000;
  wait(300,msec);
  task intakeTask(intakeThingy);
  pidLim=5000;
  ew();
  pidLim=12000;

  //turns towards the high goal
  ttarget=-850+715-27-2;
  wait(800,msec);

  //shoots the 3 discs into the high goal
  intakeTask.stop();
  task intakeTask2(intakeThingy2);
  pewpew2(3);
  intakeTask2.stop();
  // Intake.spin(fwd,-100,pct);
  // wait(300,msec);
  Intake.spin(fwd,100,pct);
  wait(700,msec);
  pewpew2(1);
  shooting=false;
}

void left_side_safe(){

  target=1200;
  task intakeTask(intakeThingy);
  shooting=true;
  ftarget=95;
  ew();
  ttarget=-250;
  etw();
  pewpew2(3);
  ttarget=-500;
  etw();
  target=1300;
  ew();

}
void left_side_danger_is_my_middle_name(){

  target=-1850;
  pidLim=12000;
  shooting=true;
  turning=false;
  ftarget=82; //fix
  wait(100,msec);
  ew();

  turning=true;
  ttarget=-330;
  etw();
  
  resetPID();
  pewpew2(2);
  shooting=false;

  ttarget=610;
  etw();
  shooting=true;
  ftarget=92;
  task intakeTask(intakeThingy);
  pidLim=7000;
  target=2000;
  wait(100,msec);
  ew();
  pidLim=12000;

  ttarget=135;
  etw();
  intakeTask.stop();
  task intakeTask2(intakeThingy2);

  pewpew2(3);
  intakeTask2.stop();

  ttarget=685;
  etw();

  pid_task.stop();
  R1.spin(forward, 100, percent);
  R2.spin(forward, 100, percent);
  R3.spin(forward, 100, percent);
  L1.spin(forward, 100, percent);
  L2.spin(forward, 100, percent);
  L3.spin(forward, 100, percent);
  wait(290, msec);
  L1.stop();
  L2.stop();
  L3.stop();

  Intake.spin(forward, -100, percent);
  //wait(320, msec);
  wait(300, msec);
  R1.stop();
  R2.stop();
  R3.stop();
  wait(500, msec);
  Intake.stop();

}

void right_side(){
  shooting=true;
  ftarget=84;
  turning=true;
  printf("wfsskfhskdjfh");
  target=100;
  wait(300,msec);
  ew();
  target=1750;
  //task intakeTask(intakeThingy);
  wait(500,msec);
  ew();
  ttarget=-800;
  wait(500,msec);
  etw();
  pewpew2(3);
  shooting=false;
  ttarget=-400;
  wait(500,msec);
  etw();

}

void win_point(){
  //start spinning the flywheel
  // move forward, turn slightly, and spin roller
  turning=true;
  ttarget=-250;
  Intake.spin(forward,-100,pct);
  wait(500,msec);
  Intake.stop();

  // turn back slighlty and back away from roller
  target=-240;
  ttarget=0;
  wait(500,msec);

  shooting=true;
  ftarget=88;
  // turn to face 3-stack of discs
  ttarget=-855;
  etw();

  // move forward and start intaking
  Intake.spin(forward,100,pct);
  task intakeTask(intakeThingy);
  target=2300;
  wait(700,msec);
  //Shooter.spin(fwd,11.2,volt);

  // continue moving forward to intake 3-stack of discs and align with high goal
  //pidLim=3000;
  wait(300,msec);
  pidLim=6000;
  ew();
  //intakeThingyWait();
  pidLim=12000;

  // turn to shoot at high goal
  ttarget=-875+625;
  wait(300,msec);
  etw();

  // shoot 3 dics
  //pewpew_auto(3,89);
  //Shooter.stop();
  pewpew2(3);
  shooting=false;

  // turn to face line of 3 discs
  ttarget=-925+30;
  wait(300,msec);
  etw();

  // move forward and intake line of 3 discs
  target=6450;
  // wait(300,msec);
  // wait(1300,msec);
  newWait();
  //intakeThingyWait2();
  //Intake.stop();
  wait(800,msec);

  //slow down as we approach the roller
  intakeTask.stop();
  Intake.stop();

  pid_task.stop();

  R1.stop();
  R2.stop();
  R3.stop();
  L1.spin(fwd,100,pct);
  L2.spin(fwd,100,pct);
  L3.spin(fwd,100,pct);
  // R1.spin(fwd,30,pct);
  // R2.spin(fwd,30,pct);
  // R3.spin(fwd,30,pct);
  wait(300,msec);
  Intake.spin(forward,-100,pct);
  wait(300,msec);
  L1.stop();
  L2.stop();
  L3.stop();
  wait(200,msec);
  Intake.stop();
  
  /*
  // turn towards roller
  ttarget=-550;
  target=6350;
  newtWait();

  // move forwards and start intake (for roller mech)
  resetPID();
  target=1200;
  Intake.spin(forward,-100,pct);

  // spin roller
  ew();
  wait(600,msec);
  Intake.stop();
  */
}

void skill(){
  //start spinning the flywheel
  // move forward, turn slightly, and spin roller
  turning=true;
  ttarget=-250;
  Intake.spin(forward,-100,pct);
  wait(800,msec);
  Intake.stop();

  // turn back slighlty and back away from roller
  target=-240;
  ttarget=0;
  wait(500,msec);

  shooting=true;
  ftarget=89;
  // turn to face 3-stack of discs
  ttarget=-875;
  etw();

  // move forward and start intaking
  Intake.spin(forward,100,pct);
  task intakeTask(intakeThingy);
  target=2300;
  wait(700,msec);
  //Shooter.spin(fwd,11.2,volt);

  // continue moving forward to intake 3-stack of discs and align with high goal
  //pidLim=3000;
  wait(300,msec);
  pidLim=6000;
  ew();
  //intakeThingyWait();
  pidLim=12000;

  // turn to shoot at high goal
  ttarget=-875+615;
  wait(300,msec);
  etw();

  // shoot 3 dics
  //pewpew_auto(3,89);
  //Shooter.stop();
  pewpew4(3);
  shooting=false;

  // turn to face line of 3 discs
  ttarget=-925;
  wait(300,msec);
  etw();

  // move forward and intake line of 3 discs
  target=6450;
  // wait(300,msec);
  // wait(1300,msec);
  newWait();
  //intakeThingyWait2();
  //Intake.stop();
  wait(500,msec);

  //slow down as we approach the roller
  intakeTask.stop();
  Intake.stop();

  pid_task.suspend();

  R1.stop();
  R2.stop();
  R3.stop();
  L1.spin(fwd,100,pct);
  L2.spin(fwd,100,pct);
  L3.spin(fwd,100,pct);
  // R1.spin(fwd,30,pct);
  // R2.spin(fwd,30,pct);
  // R3.spin(fwd,30,pct);
  Intake.spin(forward,-100,pct);
  wait(600,msec);
  L1.stop();
  L2.stop();
  L3.stop();
  wait(600,msec);
  Intake.stop();
  
  pid_task.resume();
  resetPID();
  target=-1200;
  ew();
  ttarget=-440;
  etw();
  topExpansion.set(true);
  wait(500,msec);
  resetPID();
  pidLim=6000;
  target=500;
  wait(2000,msec);
  resetPID();

}

void test(){
  shooting=true;
  ftarget=85;
  Shooter.spin(forward,11,volt);
  wait(3,sec);
  pewpew2(3);
  wait(10,sec);
  shooting=false;
  avging=false;
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
  fwTask.resume();
  L1.setStopping(coast);
  L2.setStopping(coast);
  L3.setStopping(coast);
  R1.setStopping(coast);
  R2.setStopping(coast);
  R3.setStopping(coast);
  resetPID();
  pid_task.resume();
  
  test();

  // shooting=true;
  // ftarget=90;
  // g_target2=90;
  // wait(15,sec);

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

double fwSpeed = 65;
bool revving=false;
double modifier = 1;
int fwGear = 2;
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
  if(blocker.value() && Controller1.ButtonR2.pressing()){
    blocker.set(false);
  }
  while(Controller1.ButtonR1.pressing()){
    t=0;
    while(Shooter.velocity(pct)<fwSpeed-0.5 || t<40){
      wait(10,msec);//waits until the flywheel is back up to target speed
      t++;
      if(!Controller1.ButtonR1.pressing()){
        return;
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
  switch (fwGear)
  {
    case 1:
            fwSpeed=60;
            break;
    case 2:
            fwSpeed=65;
            break;
    default:
            fwSpeed=70;
            fwGear=0;
            break;
  }
}

void incFwSpeed(){
  fwSpeed += 1;
}

void decFwSpeed(){
  fwSpeed -= 1;
}

int printSpeed(){ //prints information about the flywheel to the controller screen
  // double ps=0;
  double pt=0;
  while(true){
    // if(std::abs(Shooter.velocity(pct))-1>std::abs(ps) || std::abs(Shooter.velocity(pct))+1<std::abs(ps)){
    //   ps=Shooter.velocity(pct);
    //   Controller1.Screen.clearLine(2);
    //   Controller1.Screen.setCursor(2,1);
    //   Controller1.Screen.print(" %.2f actual",Shooter.velocity(pct));
    // } 
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

void toggleBopper(){
  dinglebopper.set(!dinglebopper.value());
}

int vibrate(){
  wait(102,sec);
  Controller1.rumble("-");
  wait(1,sec);
  // Controller1.rumble("-");
  wait(1,sec);
  // Controller1.rumble("--");
  return 1;
}

void usercontrol(void)
{
  pid_task.stop();
  fwenable=false;
  task vibrateTask(vibrate);
  //move.stop();
  Controller1.ButtonY.pressed(setFwSpeed);
  Controller1.ButtonA.pressed(incFwSpeed);
  Controller1.ButtonLeft.pressed(decFwSpeed);
  // calls the loading function
  Controller1.ButtonRight.pressed(toggleBopper);
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
      //blocker.set(false);
    }else{
      revving=false;
      Shooter.setVelocity(0,pct);
      if(!blocker.value()){
        blocker.set(true);
      }
    }
    
    // controls the intake using the left bumpers
    if(Controller1.ButtonL1.pressing()){
      Intake.setVelocity(100,pct);
    }else if(Controller1.ButtonL2.pressing()){
      Intake.setVelocity(-100,pct);
    }else{
      Intake.setVelocity(0,pct);
    }

    if(Controller1.ButtonX.pressing() && Controller1.ButtonUp.pressing()){
      topExpansion.set(true);
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
