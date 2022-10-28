/*
#include "pursuit.h"
#include <cmath>

double T=18;
int L=10;
double maxVel=200;
double maxA=10;

double distanceFormula(Point p1, Point p2){
    return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
}

std::vector<Point> addPoints(std::vector<Point> basePath, double spacing){
  std::vector<Point> newPath;
  double dx;
  double dy;double dist;
  double idx;double idy;double pointNum;
  for(auto i=basePath.begin(); i!=basePath.end()-1; ++i){
      Point p=*i;
      Point p2=*(i+1);

    dx=p2.x-p.x;
    dy=p2.y-p.y;
    
    dist=sqrt(dx*dx+dy*dy);

    pointNum=ceil(dist/spacing);

    idx=(spacing*(dx/dist));
    idy=spacing*(dy/dist);

    for(int j=0; j<pointNum; j++){
      newPath.push_back(Point(p.x+idx*j,p.y+idy*j));
    }
  }
  newPath.push_back(basePath.back());
  
  return newPath;
}

std::vector<Point> smooth(std::vector<Point> basePath, double a, double b, double tolerance){
    std::vector<Point> newPath;

    for(int i=0; i<basePath.size(); i++){
        newPath.push_back(basePath[i]);
    }
    double x;double y;
    double change=tolerance;
    while(change>=tolerance){
        change=0;
        
        for(int i=1; i<basePath.size()-1; i++){
            x=newPath[i].x;
            y=newPath[i].y;

            newPath[i].setX(x + a*(basePath[i].x-x) + b*(newPath[i-1].x+newPath[i+1].x - (2*x)));
            newPath[i].setY(y + a*(basePath[i].y-y) + b*(newPath[i-1].y+newPath[i+1].y - (2*y)));

            change+=std::abs(x-newPath[i].x)+std::abs(y-newPath[i].y);
        }
    }
    return newPath;
}

std::vector<double> getDist(std::vector<Point> path){
  double dist;
    std::vector<double> distances;
    distances.push_back(0);
    for(int i=1; i<path.size(); i++){
        dist=sqrt(pow(path[i].x-path[i-1].x,2)+pow(path[i].y-path[i-1].y,2));
        distances.push_back(distances[i-1]+dist);
    }
    return distances;
}

double findCurve(std::vector<Point> path,int pos){
    if(pos<1 || pos>=path.size()-1){
        return 0;
    }else{
        double x1=path[pos].x+0.001;double y1=path[pos].y;
        double x2=path[pos-1].x;double y2=path[pos-1].y;
        double x3=path[pos+1].x;double y3=path[pos+1].y;

        double c1=0.5*(x1*x1+y1*y1-x2*x2-y2*y2)/(x1-x2);
        double c2=(y1-y2)/(x1-x2);
        double b=0.5*(x2*x2-2*x2*c1+y2*y2-x3*x3+2*x3*c1-y3*y3)/(x3*c2-y3+y2-x2*c2);
        double a=c1-c2*b;

        double r=sqrt(pow(x1-a,2)+pow(y1-b,2));
        return 1/r;
    }
}

std::vector<double> maxVelList(std::vector<Point> path){
    //from 1 to 5
    double k=1;
    std::vector<double> velList;
    for(int i=0; i<path.size(); i++){
        velList.push_back(std::min(maxVel,k/findCurve(path,i)));
    }
    return velList;
}

std::vector<double> targetVel(std::vector<Point> path){
    std::vector<double> mvList=maxVelList(path);
    std::vector<double> targetVelList;
  double dist;
    for(int i=0; i<path.size()-1; i++){
        dist=sqrt(pow(path[i].x-path[i-1].x,2)+pow(path[i].y-path[i-1].y,2));
        targetVelList.push_back(std::min(mvList[i],sqrt(mvList[i+1]*mvList[i+1]+2*maxA*dist)));
    }

    return targetVelList;
}

int findIdx(std::vector<Point> path){
    double minDist=1000;
    int idx=0;
    double dist;
    for(int i=0; i<path.size(); i++){
        dist=sqrt(pow(path[i].x-xPosGlobal,2)+pow(path[i].y-yPosGlobal,2));
        if(dist<minDist){
            minDist=dist;
            idx=i;
        }
    }
    return idx;
}

double findCurveBot(std::vector<Point> path){
    int idx=findIdx(path);
    int a=path.size()-1;
    Point tp=path[std::min(idx+L,a)];

    return 2*(tp.x-xPosGlobal)/pow(distanceFormula(tp,Point(xPosGlobal,yPosGlobal)),2);
}

double leftVel(std::vector<Point> path,int idx){
    std::vector<double> velList=targetVel(path);
    double C=findCurveBot(path);
    double targetV=velList[idx];

    return targetV * (2+C*T)/2;
}
double rightVel(std::vector<Point> path,int idx){
    std::vector<double> velList=targetVel(path);
    double C=findCurveBot(path);
    double targetV=velList[idx];

    return targetV * (2-C*T)/2;
}
*/