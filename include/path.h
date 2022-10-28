#include "point.h"
#include <vector>
#include "odometry.h"

extern double maxV;
extern double maxA;
extern int L;
extern double T;

class Path{
    public:
        std::vector<Point> points;
    
    Path(){
        std::vector<Point> points;
    }

    Path(std::vector<Point> p){
        points=p;
    }

    void add(double a, double b){
        points.push_back(Point(a,b));
    }
    
    double dist(Point p1, Point p2){
        return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
    }

    void addPoints(double spacing){
        std::vector<Point> newPath;
        double dx;
        double dy;double d;
        double idx;double idy;double pointNum;
        for(auto i=points.begin(); i!=points.end()-1; ++i){
            Point p=*i;
            Point p2=*(i+1);

            dx=p2.x-p.x;
            dy=p2.y-p.y;
            d=dist(p,p2);

            pointNum=ceil(d/spacing);

            idx=spacing*(dx/d);
            idy=spacing*(dy/d);

            for(int j=0; j<pointNum; j++){
                newPath.push_back(Point(p.x+idx*j,p.y+idy*j));
            }
        }
        newPath.push_back(points.back());
        
        points=newPath;
    }

    void smooth(double b, double tolerance){ //b=0.75-0.98    tolerance=0.001
        double a=1-b;
        std::vector<Point> newPath;

        for(int i=0; i<points.size(); i++){
            newPath.push_back(points[i]);
        }
        double x;double y;
        double change=tolerance;
        while(change>=tolerance){
            change=0;
            
            for(int i=1; i<points.size()-1; i++){
                x=newPath[i].x;
                y=newPath[i].y;

                newPath[i].setX(x + a*(points[i].x-x) + b*(newPath[i-1].x+newPath[i+1].x - (2*x)));
                newPath[i].setY(y + a*(points[i].y-y) + b*(newPath[i-1].y+newPath[i+1].y - (2*y)));

                change+=std::abs(x-newPath[i].x)+std::abs(y-newPath[i].y);
            }
        }
        points= newPath;
    }

    double findCurve(int pos){
        if(pos<1 || pos>=points.size()-1){
            return 0;
        }else{
            double x1=points[pos].x+0.001;double y1=points[pos].y;
            double x2=points[pos-1].x;double y2=points[pos-1].y;
            double x3=points[pos+1].x;double y3=points[pos+1].y;

            double c1=0.5*(x1*x1+y1*y1-x2*x2-y2*y2)/(x1-x2);
            double c2=(y1-y2)/(x1-x2);
            double b=0.5*(x2*x2-2*x2*c1+y2*y2-x3*x3+2*x3*c1-y3*y3)/(x3*c2-y3+y2-x2*c2);
            double a=c1-c2*b;

            double r=sqrt(pow(x1-a,2)+pow(y1-b,2));
            return 1/r;
        }
    }

    std::vector<double> maxVelList(){
        //from 1 to 5
        double k=1;
        std::vector<double> velList;
        for(int i=0; i<points.size(); i++){
            velList.push_back(std::min(maxV,k/findCurve(i)));
        }
        return velList;
    }

    std::vector<double> targetVel(){
        std::vector<double> mvList=maxVelList();
        mvList.back()=0;
        std::vector<double> targetVelList;
        double d;
        for(int i=0; i<points.size()-1; i++){
            d=dist(points[i],points[i+1]);
            targetVelList.push_back(std::min(mvList[i],sqrt(pow(mvList[i+1],2)+2*maxA*d)));
        }
        

        return targetVelList;
    }

    int findIdx(){
        double minDist=1000;
        int idx=0;
        double d;
        for(int i=0; i<points.size(); i++){
            d=dist(Point(0,0),Point(points[i].x-xPosGlobal,points[i].y-yPosGlobal));
            if(d<minDist){
                minDist=d;
                idx=i;
            }
        }
        return idx;
    }

    double findCurveBot(){
        int idx=findIdx();
        int a=points.size()-1;
        Point tp=points[std::min(idx+L,a)];

        return 2*(tp.x-xPosGlobal)/pow(dist(tp,Point(xPosGlobal,yPosGlobal)),2);
    }

    double leftVel(int idx){
        std::vector<double> velList=targetVel();
        double C=findCurveBot();
        double targetV=velList[idx];

        return targetV * (2+C* T )/2;
    }
    double rightVel(int idx){
        std::vector<double> velList=targetVel();
        double C=findCurveBot();
        double targetV=velList[idx];

        return targetV * (2-C* T )/2;
    }

};