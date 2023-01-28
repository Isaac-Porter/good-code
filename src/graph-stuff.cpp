#include "graph-stuff.h"
double g_input=0;
int y,t,py;
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