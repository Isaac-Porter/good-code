#include "odometry.h"
#include "point.h"
#include <vector>

//extern std::vector<Point> path;
/*
extern int L;
extern double T;
extern double maxVel;
extern double maxA;
*/
std::vector<Point> addPoints(std::vector<Point> basePath, double spacing);
std::vector<Point> smooth(std::vector<Point> basePath, double a, double b, double tolerance);
std::vector<double> getDist(std::vector<Point> path);
double findCurve(std::vector<Point> path,int pos);
std::vector<double> maxVelList(std::vector<Point> path);
std::vector<double> targetVel(std::vector<Point> path);
double constrain(std::vector<Point> path,double x, double a, double b);
int findIdx(std::vector<Point> path);
double findCurveBot(std::vector<Point> path);
double leftVel(std::vector<Point> path,int idx);
double rightVel(std::vector<Point> path,int idx);