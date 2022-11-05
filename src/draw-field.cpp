#include "draw-field.h"
#include "string"
#include "path.h"

int goalSize = 10;

int robotSize = 15;

double lineOffset1 = 0;
double lineOffset2 = 0;

double headingX = 0;
double headingY = 0;

double robotX = 0;
double robotY = 0;

void drawPath(Path p){
    for(auto i=p.points.begin(); i!=p.points.end(); ++i){
        Point p=*i;
        Brain.Screen.setFillColor(black);
        Brain.Screen.drawCircle(p.x*1.67,240-p.y*1.67,4);
    }
}

void disc(int x, int y){
  Brain.Screen.drawCircle(x,y,6);
}
void discM(int x, int y){
  Brain.Screen.drawCircle(x,y,6);
  Brain.Screen.drawCircle(y,x,6);
}


int drawField () {
  int x = 0;
  int y = 0;
  Path p1=Path();
  p1.add(0,0);
  p1.add(0,40);
  p1.add(20,40);
  p1.addPoints(6);
  p1.smooth(0.8,0.001);

  while(1) {

    Brain.Screen.setPenWidth(1);

    Brain.Screen.setFillColor("#666666");
    Brain.Screen.drawRectangle(x, y, 240, 240);

    Brain.Screen.setPenColor("#404040");
    //horizontal lines
    for(int i = y + 40; i < y + 240; i+=40) {
      Brain.Screen.drawLine(x, i, x + 240, i);
    }

    //vertical lines
    for(int i = x + 40; i < x + 240; i+=40) {
      Brain.Screen.drawLine(i, y, i, y + 240);
    }
    drawPath(p1);

    //Field Lines
    Brain.Screen.setPenColor("#dbdbdb");
    //Home Row Lines

    Brain.Screen.drawLine(x+5, y, x + 240, y + 235);
    Brain.Screen.drawLine(x, y+5, x + 235, y + 240);

    Brain.Screen.drawLine(x,y+160,x+40,y+160);
    Brain.Screen.drawLine(x+80,y+240,x+80,y+200);
    Brain.Screen.drawLine(x+160,y,x+160,y+40);
    Brain.Screen.drawLine(x+200,y+80,x+240,y+80);

    Brain.Screen.drawLine(x,y+40,x+20,y+40);
    Brain.Screen.drawLine(x+80,y,x+80,y+20);
    Brain.Screen.drawLine(x+240,y+200,x+220,y+200);
    Brain.Screen.drawLine(x+160,y+240,x+160,y+220);

    Brain.Screen.setPenWidth(2);
    Brain.Screen.drawLine(x,y+180,x+60,y+240);
    Brain.Screen.drawLine(x+180,y,x+240,y+60);
    Brain.Screen.setPenColor("#fa2323");
    Brain.Screen.drawLine(x+80,y+160,x+40,y+160);
    Brain.Screen.drawLine(x+80,y+160,x+80,y+200);
    Brain.Screen.setPenColor("#4251f5");
    Brain.Screen.drawLine(x+160,y+80,x+160,y+40);
    Brain.Screen.drawLine(x+200,y+80,x+160,y+80);
    
    
    Brain.Screen.setPenWidth(1);
    Brain.Screen.setFillColor("#4251f5");
    Brain.Screen.setPenColor("#000000");
    Brain.Screen.drawCircle(x+30,y+210,13);

    Brain.Screen.drawRectangle(x,y+40,5,20);
    Brain.Screen.drawRectangle(x+40,y,20,5);
    Brain.Screen.drawRectangle(x+235,y+180,5,20);
    Brain.Screen.drawRectangle(x+180,y+235,20,5);

    Brain.Screen.setFillColor("#fa2323");
    Brain.Screen.drawCircle(x+210,y+30,13);

    Brain.Screen.setFillColor("#f7f12f");
    disc(20,20);
    disc(40,40);
    disc(60,60);
    disc(80,80);
    disc(100,100);
    disc(120,120);
    disc(140,140);
    disc(160,160);
    disc(180,180);
    disc(200,200);
    disc(220,220);
    discM(60,100);
    discM(100,140);
    discM(140,180);
    discM(45,152);
    discM(59,152);
    discM(73,152);
    discM(88,165);
    discM(88,180);
    discM(88,195);
    


    //Calculate offsets for box around robot
    lineOffset1 = sqrt(2) * robotSize * cos(currentAbsoluteOrientation + M_PI_4);
    lineOffset2 = sqrt(2) * robotSize * cos(currentAbsoluteOrientation - M_PI_4);

    robotX = xPosGlobal * 1.67;
    robotY = -yPosGlobal * 1.67;

    //Draw the Robot
    Brain.Screen.setPenColor(black);
    Brain.Screen.drawLine(robotX + lineOffset1, 240 + robotY - lineOffset2, robotX + lineOffset2, 240 + robotY + lineOffset1);
    Brain.Screen.drawLine(robotX + lineOffset2, 240 + robotY + lineOffset1, robotX - lineOffset1, 240 + robotY + lineOffset2);
    Brain.Screen.drawLine(robotX - lineOffset1, 240 + robotY + lineOffset2, robotX - lineOffset2, 240 + robotY - lineOffset1);
    Brain.Screen.drawLine(robotX - lineOffset2, 240 + robotY - lineOffset1, robotX + lineOffset1, 240 + robotY - lineOffset2);

    //calculate where to place forward line
    headingX = 10 * cos(currentAbsoluteOrientation);
    headingY = 10 * sin(currentAbsoluteOrientation);

    //drawPath(newPath);
    //Draw Heading Line
    Brain.Screen.drawLine(robotX, 240 + robotY, robotX + headingX, 240 + robotY - headingY);

    Brain.Screen.setPenColor(white);
    Brain.Screen.setFillColor(black);
    Brain.Screen.printAt(250,20,"left front: %8.3f",L1.temperature(temperatureUnits::celsius));
    Brain.Screen.printAt(250,40,"left mid: %8.3f",L2.temperature(temperatureUnits::celsius));
    Brain.Screen.printAt(250,60,"left back: %8.3f",L3.temperature(temperatureUnits::celsius));
    Brain.Screen.printAt(250,80,"right front: %8.3f",R1.temperature(temperatureUnits::celsius));
    Brain.Screen.printAt(250,100,"right mid: %8.3f",R2.temperature(temperatureUnits::celsius));
    Brain.Screen.printAt(250,120,"right back: %8.3f",R3.temperature(temperatureUnits::celsius));
    Brain.Screen.printAt(250,140,"Intake: %8.3f",Intake.temperature(temperatureUnits::celsius));
    Brain.Screen.printAt(250,160,"Shooter: %8.3f",Shooter.temperature(temperatureUnits::celsius));
  
    Brain.Screen.printAt(250,190,"X: %f",xPosGlobal);
    Brain.Screen.printAt(250,210,"Y: %f",yPosGlobal);
    Brain.Screen.printAt(250,230,"angle: %f",currentAbsoluteOrientation*180/M_PI);

    task::sleep(20);
  }

  return 1;
}