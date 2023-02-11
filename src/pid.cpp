#include "pid.h"

double kp=15,ki=0.1,kd=5;//5
double integral=0,derivative=0;
double error=0,prevError=0,target=0,output=0,input=0;

double kpt=15,kit=0.01,kdt=10;
double tintegral=0,tderivative=0;
double terror=0,tprevError=0,ttarget=0, toutput=0, tinput=0;

bool atTarget=false;
bool atAngle=false;

bool turning=true;

double pidLim=12000;
bool pid_enable=true;

int pid(){
  //Controller1.Screen.clearScreen();
  L1.setPosition(0,degrees);
  L2.setPosition(0,degrees);
  L3.setPosition(0,degrees);
  R1.setPosition(0,degrees);
  R2.setPosition(0,degrees);
  R3.setPosition(0,degrees);
  target=0;
  ttarget=0;
  
  while(true){
    if(pid_enable){
    input=(L1.position(degrees)+L2.position(degrees)+L3.position(degrees)+R1.position(degrees)+R2.position(degrees)+R3.position(degrees))/6;
    error=target-input;
    if(prevError==0 && error!=0){
      prevError=error;
    }
    integral+=error;
    derivative=error-prevError;
    prevError=error;
    
    output=kp*error+ki*integral+kd*derivative;

    
    tinput=(L1.position(degrees)+L2.position(degrees)+L3.position(degrees)-R1.position(degrees)-R2.position(degrees)-R3.position(degrees))/6;
    terror=ttarget-tinput;
    if(turning){
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
    Controller1.Screen.print("%f",(output-toutput)/1000);
    printf("%f\n",(output-toutput)/1000);

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

    wait((int)10,msec);

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
    }
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