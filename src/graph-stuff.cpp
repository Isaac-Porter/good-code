#include "graph-stuff.h"
#include "pid.h"
double g_target;
double g_input=0;
int y,t,py;

bool shot=false;

int graph(){
    //t=0;
    g_input=0;
    py=0;
    Brain.Screen.clearScreen();
    Brain.Screen.setOrigin(0,240);
    Brain.Screen.setPenColor(green);
    Brain.Screen.drawLine(0,-168,480,-168);
    double scale=168.0/g_target;
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawPixel(1,-1);
    L1.setPosition(0,degrees);
    L2.setPosition(0,degrees);
    L3.setPosition(0,degrees);
    R1.setPosition(0,degrees);
    R2.setPosition(0,degrees);
    R3.setPosition(0,degrees);
    while(true){
        Brain.Screen.printAt(20,-220, "%.3f",g_input);
        scale=168.0/g_target;
        g_input=1+(L1.position(degrees)+L2.position(degrees)+L3.position(degrees)+R1.position(degrees)+R2.position(degrees)+R3.position(degrees))/6;
        y=-(scale*g_input);
        Brain.Screen.drawLine(t,y,t-1,py);
        py=y;
        t+=1;
        wait(70,msec);
    }

    return 1;
}

int tgraph(){
    t=0;
    g_input=0;
    py=0;
    Brain.Screen.clearScreen();
    Brain.Screen.setOrigin(0,240);
    Brain.Screen.setPenColor(green);
    Brain.Screen.drawLine(0,-168,480,-168);
    double scale=168.0/g_target;
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawPixel(1,-1);
    L1.setPosition(0,degrees);
    L2.setPosition(0,degrees);
    L3.setPosition(0,degrees);
    R1.setPosition(0,degrees);
    R2.setPosition(0,degrees);
    R3.setPosition(0,degrees);
    while(true){
        Brain.Screen.printAt(20,-220, "%.3f",g_input);
        scale=168.0/g_target;
        g_input=1+(L1.position(degrees)+L2.position(degrees)+L3.position(degrees)-R1.position(degrees)-R2.position(degrees)-R3.position(degrees))/6;
        y=-(scale*g_input);
        Brain.Screen.drawLine(t,y,t-1,py);
        py=y;
        t+=1;
        wait(70,msec);
    }

    return 1;
}

double g_target2=85;
double average=0;
double g_input3=0;
int count2=0;
int count3=0;
int moving_average_size=10;
double prev_values[10];

double g_error=0;


int fgraph(){
    t=0;
    g_target=12;
    double g_input2=0;
    double p=0;
    double pp=0;
    double y2=0;
    double py2=0;
    double y3;
    double py3;
    double y4;
    double py4;
    py=0;
    double scale3=25;
    Brain.Screen.clearScreen();
    Brain.Screen.setOrigin(0,240);
    Brain.Screen.setPenColor(green);
    Brain.Screen.drawLine(0,-168,480,-168);
    Brain.Screen.drawLine(0,-50,480,-50);
    Brain.Screen.setPenColor(vex::color::orange);
    Brain.Screen.drawLine(0,-50+scale3*0.5,480,-50+scale3*0.5);
    Brain.Screen.drawLine(0,-50-scale3*0.5,480,-50-scale3*0.5);

    //Brain.Screen.drawLine(250,0,250,-240);
    double scale=168.0/g_target;
    double scale2=168.0/g_target2;
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawPixel(1,-1);
    L1.setPosition(0,degrees);
    L2.setPosition(0,degrees);
    L3.setPosition(0,degrees);
    R1.setPosition(0,degrees);
    R2.setPosition(0,degrees);
    R3.setPosition(0,degrees);
    
    timer timey;
    double timeMoment=0;
    double pin,rev_input;
    while(true){
        g_error=ferror1;
        
        if(avging){
            count2++;
            average+=g_input2;
        }
        if(count3<moving_average_size){
            count3++;
        }
        if(shot){
            shot=false;
            Brain.Screen.setPenColor(blue);
            Brain.Screen.drawLine(t,-240,t,0);
            Brain.Screen.setPenColor(white);
        }
        pin=g_input2;
        // if(std::abs(g_input2-pin)>0.125){
        //     rev_input=pin+0.125*((g_input2-pin)/std::abs(g_input2-pin));
        //     pin=rev_input;
        // }else{
            rev_input=g_input2;
        // }
        
        // if(Shooter.velocity(pct)>g_target2-3){
        //     g_input=Shooter.velocity(pct)/9+(g_target2-Shooter.velocity(pct))/2;
        // }else{
        //     g_input=12;
        // }

        double number=0.01;
        g_input3=0;
        for(int i=0; i<moving_average_size; i++){
            if(prev_values[i]==0){
                break;
            }
            number++;
            g_input3+=prev_values[i];
        }
        g_input3/=number;

        prev_values[0]=g_input2;

        for(int i=moving_average_size-2; i>=0; i--){
            prev_values[i+1]=prev_values[i];
        }


        g_input=Shooter.voltage(volt);
        g_input2=Shooter.velocity(pct);
        timeMoment=timey.time();
        Brain.Screen.printAt(20,-220, "%.1f",g_input);
        Brain.Screen.printAt(270,-220, "%.1f",g_input2);
        Brain.Screen.printAt(200,-220, "%.1f",g_input3);
        Brain.Screen.printAt(350,-220, "%.1f",average/count2);
        if(shooting){
            Brain.Screen.printAt(350,-190, "%.1f",timeMoment);
        }
        scale=168.0/g_target;
        p=Shooter.position(rev)*1200;
        //g_input=p-pp;
        pp=p;
        y=-(scale*g_input);
        Brain.Screen.setPenColor(yellow);
        Brain.Screen.drawLine(t,y,t-1,py);
        Brain.Screen.setPenColor(white);
        py=y;
        scale2=168.0/g_target2;
        y2=-(scale2*rev_input);
        Brain.Screen.drawLine(t,y2,t-1,py2);
        py2=y2;
        y3=-(scale2*g_input3);
        Brain.Screen.setPenColor(red);
        Brain.Screen.drawLine(t,y3,t-1,py3);
        Brain.Screen.setPenColor(white);
        py3=y3;
        y4=-(50+g_error*scale3);
        Brain.Screen.setPenColor(cyan);
        Brain.Screen.drawLine(t,y4,t-1,py4);
        Brain.Screen.setPenColor(white);
        py4=y4;
        t+=1;
        while(timey.time(timeUnits::msec)<timeMoment+50){
            vex::task::sleep(1);
        }
    }

    return 1;
}