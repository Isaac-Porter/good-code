#include "chassis_control.h"
#include "vex.h"
#include "odometry.h"
#include "path.h"

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


//double kv=1/200, ka=0.002, kp=0.01;
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