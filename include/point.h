
class Point{
  public:
    double x;
    double y;

    Point(double a, double b){
      x=a;
      y=b;
    }

    Point(){
      x=0;
      y=0;
    }

    double getX(){
      return x;
    }
    double getY(){
      return y;
    }
    double* getPoint(){
      double p[2]={x,y};
      double *point=p;
      return point;
    }

    void setX(double a){
        x=a;
    }
    void setY(double a){
        y=a;
    }
};