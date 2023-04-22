#include "pid.h"
#include "graph-stuff.h"

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

  timer timey;
  double timeMoment=0;
  
  while(true){
    timeMoment=timey.time(timeUnits::msec);
    //printf("%f\n", output);
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
      //printf("%f\n",(output-toutput)/1000);

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
    while(timey.time(timeUnits::msec)<timeMoment+10){
      vex::task::sleep(1);
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
  output=0;
  toutput=0;
  derivative=0;
  tderivative=0;
}

bool stopping=false;

void stopFW(){
  double power=12;
  while(power>-2){
    Shooter.spin(forward,power,volt);
    power-=0.5;
    wait(20,msec);
    printf("stopping");
  }
  Shooter.stop();
  stopping=false;
}

bool fwenable=true;
bool shooting=false;
bool avging=false;
double fwpi=0, fwpp=0;

double fkp=500,fki=1.05,fkd=10; // double fkp=500,fki=1.05,fkd=10;
double fintegral=0,fderivative=0;
double ferror1=0,fprevError=0,ftarget=0,foutput=0,finput=0;
double pout=0;
double output_revised=0;
bool rev_initial=true;
bool just_started=false;

double pin, rev_input;

int intialTime=2000;
int timeElapsed=0;

int flywheel_pid(){

  Shooter.setPosition(0,rev);
  timer timey;
  double timeMoment=0;
  while(fwenable){
    timeMoment=timey.time(timeUnits::msec);
    //fwpi=Shooter.position(rotationUnits::rev)*240;
    //finput=fwpi-fwpp;
    pin=rev_input;

    finput=Shooter.velocity(pct);
    if(std::abs(finput-g_input3)<2){
      rev_input=finput;
    }else{
      rev_input=g_input3;
    }

    if(shooting && rev_initial){

      Shooter.spin(fwd, 12, volt);
      ferror1=ftarget-rev_input;
      if(fprevError==0 && ferror1!=0){
        fprevError=ferror1;
      }
      fintegral+=ferror1;
      
      fderivative=ferror1-fprevError;
      fprevError=ferror1;
      //Shooter.spin(fwd,11,volt);
      //Shooter.spin(fwd,90,pct);

      if(rev_input>ftarget-5){
        // Shooter.spin(fwd,rev_input/8+(ftarget-rev_input)/1.5,volt);
        fintegral=ftarget*200-7000;
        rev_initial=false;
        avging=true;
      }else{
        wait(100,msec);
        timeElapsed+=100;
        // Shooter.spin(fwd,12,volt);
      }

    }
    if(!shooting && !rev_initial){
      Shooter.stop();
      rev_initial=true;
      timeElapsed=0;
    }
    if(shooting && !rev_initial){
      //g_target=fwSpeed;
      //ftarget=fwSpeed;

      ferror1=ftarget-rev_input;

      if(fprevError==0 && ferror1!=0){
        fprevError=ferror1;
      }
      fintegral+=ferror1;
      /*
      if( fintegral < -12000){
        fintegral=-12000;
      }else if(fintegral > 12000){
        fintegral=12000;
      }*/
      
      fderivative=ferror1-fprevError;
      fprevError=ferror1;
      
      pout=foutput;
      foutput=fkp*ferror1+fki*fintegral+fkd*fderivative;
      
      
      // if(std::abs(foutput-pout)>5){
      //   output_revised=pout+5*((foutput-pout)/std::abs(foutput-pout));
      // }else{
      //   output_revised=foutput;
      // }
      output_revised=foutput;

      Shooter.spin(forward,output_revised/1000,volt);
    }
      //Controller1.Screen.clearLine(3);
      //Controller1.Screen.setCursor(3, 1);
      //Controller1.Screen.print("%f",timeMoment);
    while(timey.time(timeUnits::msec)<timeMoment+100){
      vex::task::sleep(1);
    }
  }
  return 1;
}