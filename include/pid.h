#include "vex.h"

extern double target;
extern double ttarget;
extern bool turning;
extern bool atTarget;
extern bool atAngle;
extern double pidLim;
extern bool pid_enable;
extern double derivative;
extern double error;
extern double tderivative;
extern double terror;

int pid();
void resetPID();